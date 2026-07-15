"""detector_node — детекция цели в кадре камеры (Фаза 3, инкремент 2).

Поток: /camera/image (sensor_msgs/Image, rgb8) → cv_bridge (RGB→BGR) → YOLO
(ObjectDetector) → выбор ОДНОЙ цели нужного класса → нормализованные offset/area →
publish /perception/target (drone_interfaces/Target). Опционально публикует
/perception/image — кадр с нарисованным bbox для визуальной проверки.

Координаты в Target нормализованы [-1..1] и не зависят от разрешения камеры
(см. docs/phase3_setup.md §7). Контроллер (follower_node, инкремент 3) рулит по ним.
Все настройки — ros2-параметры (значения в configs/perception/detector.yaml), без
магических чисел в коде (CLAUDE.md).
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from drone_interfaces.msg import Target

from drone_perception.detector import ObjectDetector


class DetectorNode(Node):
    """Детектор цели: кадр → bbox → нормализованное смещение от центра."""

    def __init__(self):
        super().__init__('detector_node')

        # --- Параметры (значения по умолчанию переопределяются из yaml) ---
        self._image_topic = self.declare_parameter('image_topic', '/camera/image').value
        self._camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera/camera_info').value
        self._target_topic = self.declare_parameter('target_topic', '/perception/target').value
        self._annotated_topic = self.declare_parameter('annotated_topic', '/perception/image').value
        self._publish_annotated = self.declare_parameter('publish_annotated', True).value
        model_path = self.declare_parameter('model_path', 'yolov8n.pt').value
        confidence = self.declare_parameter('confidence_threshold', 0.5).value
        device = self.declare_parameter('device', 'cuda').value
        # Какие классы считаем целью (COCO-имена). Пусто → любой класс.
        self._target_classes = list(self.declare_parameter('target_classes', ['person']).value)
        # Критерий выбора одной цели из нескольких: "largest" | "closest".
        # Используется только в detect-only (tracking=False, откат к Фазе 3).
        self._selection = self.declare_parameter('selection', 'largest').value
        # Фаза 4 (Ф4-1): трекинг ByteTrack и политика захвата одного track_id.
        tracking = bool(self.declare_parameter('tracking', True).value)
        tracker = self.declare_parameter('tracker', 'bytetrack.yaml').value
        # Диагностика трекинга (dets/ids/locked/matched, троттлинг 1 с). Выключено по
        # умолчанию — инструмент отладки M1/M3, включать при разборе поведения трекера.
        self._debug_tracking = bool(self.declare_parameter('debug_tracking', False).value)

        # Pinhole-дистанция (Фаза 4, M3 / Ф4-5): distance_m = f_y·H_real/h_px.
        # f_y берём из /camera/camera_info (не хардкодим). H_real — известный рост цели.
        self._h_real = float(self.declare_parameter('h_real_m', 1.7).value)
        # EMA-сглаживание шумной оценки (R2): alpha∈(0..1], 1.0 = без сглаживания.
        self._dist_alpha = float(self.declare_parameter('distance_ema_alpha', 0.4).value)
        # Доверяем дистанции, только если bbox целиком в кадре (R2): иначе h_px занижен
        # (голова/ноги срезаны) → дистанция завышена. Отступ от края, пиксели.
        self._dist_edge_margin = int(self.declare_parameter('distance_edge_margin_px', 3).value)

        self._bridge = CvBridge()
        self.get_logger().info(
            f"Загружаю YOLO: model={model_path} device={device} conf={confidence} "
            f"tracking={tracking} tracker={tracker}")
        self._detector = ObjectDetector(
            model_path=model_path,
            confidence_threshold=confidence,
            device=device,
            tracking=tracking,
            tracker=tracker,
        )

        # id залоченной цели (Ф4-2). None = ещё не захватили. Захватываем крупнейший
        # подтверждённый трек один раз, далее держим ИМЕННО его; при потере id в кадре
        # публикуем detected=false (перелов/relock — забота FSM, Инкремент 7). id при этом
        # не сбрасываем: если ByteTrack вернёт тот же трек, слежение продолжится.
        self._locked_id = None

        # Pinhole-состояние: f_y из camera_info и текущая EMA-дистанция.
        self._fy = None            # фокус (пиксели) по вертикали из camera_info.k[4]
        self._dist_ema = None      # сглаженная дистанция; None = нет валидной оценки

        # Камера — sensor data (best-effort), поэтому такой же QoS на подписке.
        self._sub = self.create_subscription(
            Image, self._image_topic, self._on_image, qos_profile_sensor_data)
        self._info_sub = self.create_subscription(
            CameraInfo, self._camera_info_topic, self._on_camera_info,
            qos_profile_sensor_data)
        self._target_pub = self.create_publisher(Target, self._target_topic, 10)
        self._annotated_pub = (
            self.create_publisher(Image, self._annotated_topic, 10)
            if self._publish_annotated else None)

        self.get_logger().info(
            f"detector_node готов: {self._image_topic} → {self._target_topic}"
            f" (классы={self._target_classes or 'любые'}, выбор={self._selection})")

    def _on_image(self, msg: Image):
        # ROS rgb8 → OpenCV BGR (иначе YOLO деградирует, см. §6).
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]

        detections = self._detector.detect(frame)
        # Фильтр по целевым классам (пусто → берём все).
        if self._target_classes:
            detections = [d for d in detections if d[1] in self._target_classes]

        target_msg = Target()
        target_msg.header = msg.header  # стамп кадра — для контроля свежести в контроллере

        target = self._pick_target(detections, width, height)

        # Диагностика M1/M3 (троттлинг 1 с, за флагом debug_tracking): сколько детекций,
        # какие track_id видны, на кого залочены, совпал ли залоченный. Различает «текучие
        # id» (ids растут, matched=False) от «нет детекций» (dets=0). Выключено по умолчанию.
        if self._debug_tracking:
            ids = sorted(d[3] for d in detections if d[3] is not None)
            self.get_logger().info(
                f"dets={len(detections)} ids={ids} locked={self._locked_id} "
                f"matched={target is not None}",
                throttle_duration_sec=1.0)

        distance_m = 0.0
        if target is not None:
            (x1, y1, x2, y2), class_name, conf, track_id = target
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            distance_m = self._estimate_distance((x1, y1, x2, y2), width, height)
            target_msg.detected = True
            target_msg.offset_x = float((cx - width / 2.0) / (width / 2.0))
            target_msg.offset_y = float((cy - height / 2.0) / (height / 2.0))
            target_msg.area_ratio = float(((x2 - x1) * (y2 - y1)) / (width * height))
            target_msg.track_id = int(track_id) if track_id is not None else -1
            target_msg.distance_m = float(distance_m)
        else:
            self._dist_ema = None  # цель ушла — не тянем устаревшую EMA-дистанцию
            target_msg.detected = False
            target_msg.offset_x = 0.0
            target_msg.offset_y = 0.0
            target_msg.area_ratio = 0.0
            target_msg.track_id = -1
            target_msg.distance_m = 0.0

        self._target_pub.publish(target_msg)

        if self._annotated_pub is not None:
            self._publish_overlay(frame, detections, target, distance_m, width, height, msg.header)

    def _on_camera_info(self, msg: CameraInfo):
        # K = [fx 0 cx; 0 fy cy; 0 0 1] (row-major). f_y = K[4]. Наклон камеры —
        # extrinsic, intrinsics не меняет, поэтому одного чтения достаточно.
        fy = float(msg.k[4])
        if fy > 0.0 and self._fy is None:
            self._fy = fy
            self.get_logger().info(f"camera_info: f_y={fy:.1f} px (pinhole-дистанция активна)")

    def _estimate_distance(self, bbox, width, height):
        """Pinhole-дистанция до цели (Ф4-5): distance = f_y·H_real/h_px, с EMA.

        Возвращает метры или 0.0, если оценка недоступна/ненадёжна: нет f_y, нулевая
        высота bbox, или bbox касается края кадра (голова/ноги срезаны → h_px занижен,
        дистанция завышена — R2). При разрыве надёжности EMA сбрасывается."""
        if self._fy is None:
            return 0.0
        x1, y1, x2, y2 = bbox
        h_px = y2 - y1
        m = self._dist_edge_margin
        full_in_frame = (x1 >= m and y1 >= m and x2 <= width - m and y2 <= height - m)
        if h_px <= 0 or not full_in_frame:
            self._dist_ema = None  # ненадёжно — рвём сглаживание, чтобы не тянуть артефакт
            return 0.0
        raw = self._fy * self._h_real / float(h_px)
        # EMA: seed сырым значением при первом надёжном измерении, далее сглаживаем.
        if self._dist_ema is None:
            self._dist_ema = raw
        else:
            self._dist_ema = self._dist_alpha * raw + (1.0 - self._dist_alpha) * self._dist_ema
        return self._dist_ema

    def _pick_target(self, detections, width, height):
        """Выбрать залоченную цель.

        Трекинг (Ф4-2): держим self._locked_id. Ещё не захватили → лочим крупнейший
        ПОДТВЕРЖДЁННЫЙ трек (у которого есть track_id). Захватили → возвращаем детекцию
        с этим id; нет его в кадре → None (потеря; id держим, перелов = FSM/Инкр. 7).
        Detect-only (tracking=False) → откат к выбору Фазы 3 (_select)."""
        if not self._detector.tracking:
            return self._select(detections, width, height)

        # Только треки с подтверждённым id (ByteTrack назначает не в первом же кадре).
        tracked = [d for d in detections if d[3] is not None]

        if self._locked_id is not None:
            for d in tracked:
                if d[3] == self._locked_id:
                    return d
            return None  # залоченный id не виден в этом кадре — цель потеряна

        # Захвата ещё нет — берём крупнейший подтверждённый трек и лочимся на него.
        if not tracked:
            return None
        target = max(tracked, key=lambda d: (d[0][2] - d[0][0]) * (d[0][3] - d[0][1]))
        self._locked_id = target[3]
        self.get_logger().info(f"Захват цели: track_id={self._locked_id}")
        return target

    def _select(self, detections, width, height):
        """Выбрать одну цель из нескольких (detect-only, откат Фазы 3). largest =
        крупнейший bbox, closest = ближайший к центру кадра."""
        if not detections:
            return None
        if self._selection == 'closest':
            fx, fy = width / 2.0, height / 2.0

            def dist2(d):
                (x1, y1, x2, y2), _, _, _ = d
                return ((x1 + x2) / 2.0 - fx) ** 2 + ((y1 + y2) / 2.0 - fy) ** 2

            return min(detections, key=dist2)
        # default: largest
        return max(detections, key=lambda d: (d[0][2] - d[0][0]) * (d[0][3] - d[0][1]))

    def _publish_overlay(self, frame, detections, target, distance_m, width, height, header):
        # Центр кадра (цель наведения) — зелёный крест.
        cx0, cy0 = width // 2, height // 2
        cv2.drawMarker(frame, (cx0, cy0), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
        # Мини-оверлей: рисуем ВСЕ треки, залоченный выделяем (красный/толстый + линия
        # к центру), прочие — синим/тонким (BGR: синий=(255,0,0)). У залоченного — дистанция
        # (M3). Полный оверлей (FSM/FPS) — M6.
        for d in detections:
            (x1, y1, x2, y2), class_name, conf, track_id = d
            locked = d is target
            color = (0, 0, 255) if locked else (255, 0, 0)
            thickness = 2 if locked else 1
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
            id_txt = f"id{track_id}" if track_id is not None else "id?"
            label = f"{id_txt} {class_name} {conf:.2f}"
            if locked:
                label += f" {distance_m:.1f}m" if distance_m > 0.0 else " d=?"
            cv2.putText(frame, label, (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            if locked:
                cv2.line(frame, (cx0, cy0), ((x1 + x2) // 2, (y1 + y2) // 2), color, 1)
        out = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = header
        self._annotated_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
