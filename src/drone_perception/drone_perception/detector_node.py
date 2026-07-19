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

import math

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from drone_interfaces.msg import Target

from drone_perception.detector import ObjectDetector
from drone_perception.reid import ObjectEmbedder, TargetIdentity
from drone_perception.reid.embedder import crop_bbox


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
        # f_y берём из /camera/camera_info; если он не приходит — считаем из известного
        # horizontal_fov и ширины кадра: f_y=(width/2)/tan(hfov/2) (камера квадратнопиксельная).
        self._h_real = float(self.declare_parameter('h_real_m', 1.7).value)
        self._hfov = float(self.declare_parameter('horizontal_fov_rad', 1.74).value)
        # EMA-сглаживание шумной оценки (R2): alpha∈(0..1], 1.0 = без сглаживания.
        self._dist_alpha = float(self.declare_parameter('distance_ema_alpha', 0.4).value)
        # Доверяем дистанции, только если bbox целиком в кадре (R2): иначе h_px занижен
        # (голова/ноги срезаны) → дистанция завышена. Отступ от края, пиксели.
        self._dist_edge_margin = int(self.declare_parameter('distance_edge_margin_px', 3).value)

        # --- Ре-идентификация цели (Фаза 4, M2 / Ф4-13) ---
        # Лок держится на ПОСТОЯННОЙ идентичности (эталон облика), а не на временном
        # track_id: последний умирает при окклюзии и возвращается другим числом.
        reid_enabled = bool(self.declare_parameter('reid.enabled', True).value)
        self._reid_refresh_every = int(self.declare_parameter('reid.refresh_every_n_frames', 15).value)
        self._reid_relock_period = float(self.declare_parameter('reid.relock_check_interval_s', 0.4).value)
        self._reid_crop_padding = int(self.declare_parameter('reid.crop_padding_px', 6).value)
        self._reid_min_bbox_px = int(self.declare_parameter('reid.min_bbox_height_px', 40).value)
        self._reid_confirm_after_s = float(
            self.declare_parameter('reid.confirm_after_s', 3.0).value)
        self._reid_debug = bool(self.declare_parameter('reid.debug', False).value)

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

        # track_id залоченной цели (Ф4-2). Это ВСПОМОГАТЕЛЬНОЕ, быстрое звено: пока трекер
        # ведёт цель, сверять эмбеддинги каждый кадр незачем. Настоящий якорь личности —
        # self._identity (Ф4-13). При смерти трека track_id обнуляется, идентичность живёт.
        self._locked_id = None

        # Эталон облика цели + гейт «это он?» (Ф4-13). Эмбеддер грузится один раз; если
        # open-clip-torch/веса недоступны — ReID выключается, а детектор Фазы 3 продолжает
        # работать (тогда поведение = M1: потерял track_id → потерял цель).
        self._embedder = None
        self._identity = None
        if reid_enabled:
            self._embedder = ObjectEmbedder(
                model_name=self.declare_parameter('reid.model_name', 'ViT-B-32').value,
                pretrained=self.declare_parameter('reid.pretrained', 'laion2b_s34b_b79k').value,
                device=self.declare_parameter('reid.device', 'cuda').value,
                logger=self.get_logger())
            self._identity = TargetIdentity(
                similarity_threshold=self.declare_parameter('reid.similarity_threshold', 0.90).value,
                max_embeddings=self.declare_parameter('reid.max_embeddings', 5).value,
                aggregation=self.declare_parameter('reid.aggregation', 'mean').value,
                ema_alpha=self.declare_parameter('reid.ema_alpha', 0.3).value,
                recent_n=self.declare_parameter('reid.recent_n', 3).value,
                weighted_decay=self.declare_parameter('reid.weighted_decay', 0.7).value)
        else:
            self.get_logger().warn(
                'ReID выключен параметром reid.enabled=false — перезахват цели после '
                'потери трека невозможен (поведение M1).')

        self._frame_idx = 0            # счётчик кадров для скважности подпитки эталона
        self._last_relock_try = None   # время последней попытки перезахвата (троттлинг)
        self._last_score = None        # балл сходства последней сверки — для оверлея/логов
        # Предварительный vs подтверждённый лок. Строгий запрет подмены цели (Ф4-11) имеет
        # смысл только когда мы ДЕЙСТВИТЕЛЬНО кого-то вели: если эталон снят с плохого кадра
        # и цель сразу потерялась, «верность» ему запирает дрона навсегда. Поэтому лок
        # считается подтверждённым лишь после непрерывного слежения confirm_after_s.
        self._lock_started = None
        self._identity_confirmed = False

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

        self._frame_idx += 1
        target = self._pick_target(frame, detections, width, height)

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

        Возвращает метры или 0.0, если оценка ненадёжна: нулевая высота bbox или bbox
        касается края кадра (голова/ноги срезаны → h_px занижен, дистанция завышена — R2).
        При разрыве надёжности EMA сбрасывается."""
        # f_y: из camera_info, иначе из горизонтального FOV и ширины кадра (fx=fy).
        fy = self._fy if self._fy is not None else (width / 2.0) / math.tan(self._hfov / 2.0)
        x1, y1, x2, y2 = bbox
        h_px = y2 - y1
        m = self._dist_edge_margin
        full_in_frame = (x1 >= m and y1 >= m and x2 <= width - m and y2 <= height - m)
        if h_px <= 0 or not full_in_frame:
            self._dist_ema = None  # ненадёжно — рвём сглаживание, чтобы не тянуть артефакт
            return 0.0
        raw = fy * self._h_real / float(h_px)
        # EMA: seed сырым значением при первом надёжном измерении, далее сглаживаем.
        if self._dist_ema is None:
            self._dist_ema = raw
        else:
            self._dist_ema = self._dist_alpha * raw + (1.0 - self._dist_alpha) * self._dist_ema
        return self._dist_ema

    def _pick_target(self, frame, detections, width, height):
        """Выбрать залоченную цель. Двухуровневая схема (Ф4-13).

        Быстрый уровень — трекер: пока `self._locked_id` виден в кадре, это наша цель,
        эмбеддинги не считаем (бюджет GPU, Ф4-15), лишь изредка подпитываем эталон.
        Медленный уровень — идентичность: трек умер → сверяем КАЖДОГО кандидата с эталоном
        облика и перезахватываемся ТОЛЬКО на прошедшего порог. Не прошёл никто → None,
        то есть «свой не найден», дрон продолжит искать (Ф4-11) и НИКОГДА не поедет за чужим.

        Detect-only (tracking=False) → откат к выбору Фазы 3 (_select)."""
        if not self._detector.tracking:
            return self._select(detections, width, height)

        # Только треки с подтверждённым id (ByteTrack назначает не в первом же кадре).
        tracked = [d for d in detections if d[3] is not None]

        # --- Цель ведётся трекером: самый дешёвый путь ---
        if self._locked_id is not None:
            for d in tracked:
                if d[3] == self._locked_id:
                    self._maybe_reinforce(frame, d)
                    self._maybe_confirm()
                    return d
            # Трек умер. НЕ сбрасываем идентичность — только вспомогательный track_id.
            if self._identity_confirmed:
                self.get_logger().warn(
                    f"Трек цели потерян (был track_id={self._locked_id}). "
                    f"{'Ищу своего по облику (ReID).' if self._reid_active() else 'ReID недоступен — перезахват невозможен.'}")
            else:
                # Лок так и не подтвердился — вести было по сути некого. Начинаем заново,
                # это НЕ подмена цели: своего человека мы ещё не выбрали.
                self.get_logger().warn(
                    f"Трек track_id={self._locked_id} потерян до подтверждения лока "
                    f"({self._reid_confirm_after_s} с) — эталон сбрасываю, захватываю заново.")
                if self._identity is not None:
                    self._identity.locked = False
            self._locked_id = None

        if not tracked:
            return None

        # --- Первичный захват: крупнейший (= ближайший) трек, и запоминаем его облик ---
        # Пока лок ПРЕДВАРИТЕЛЬНЫЙ (не подтверждён устойчивым слежением), потеря трека
        # разрешает перезахват заново — см. _confirmed(). Иначе один неудачный эталон,
        # снятый на взлёте с крошечной смазанной фигуры, запирал бы систему навсегда.
        if not self._reid_active() or not self._confirmed():
            target = max(tracked, key=lambda d: (d[0][2] - d[0][0]) * (d[0][3] - d[0][1]))
            self._locked_id = target[3]
            self._lock_started = self.get_clock().now()
            if self._reid_active():
                # Эталон снимаем ТОЛЬКО с достоверного кропа. Мелкий bbox (далеко/на
                # взлёте) даёт мусорный эталон, с которым потом ничего не совпадает —
                # ровно этим система и запиралась. Не годится — запишем дозахватом.
                if (target[0][3] - target[0][1]) >= self._reid_min_bbox_px:
                    emb = self._embed_one(frame, target[0])
                else:
                    emb = None
                if emb is not None:
                    self._identity.lock(emb)
                    self.get_logger().info(
                        f"Захват цели: track_id={self._locked_id}, облик запомнен (ReID).")
                else:
                    self.get_logger().info(
                        f"Захват цели: track_id={self._locked_id}; облик пока не записан "
                        f"(bbox мелкий), жду годного кадра.")
            else:
                self.get_logger().info(f"Захват цели: track_id={self._locked_id} (без ReID).")
            return target

        # --- Перезахват: сверяем кандидатов с эталоном ---
        return self._relock(frame, tracked)

    def _reid_active(self):
        return (self._embedder is not None and self._embedder.available
                and self._identity is not None)

    def _confirmed(self):
        """Есть ли ПОДТВЕРЖДЁННАЯ цель — та, за которую действует запрет подмены (Ф4-11)."""
        return (self._identity is not None and self._identity.locked
                and self._identity_confirmed)

    def _maybe_confirm(self):
        """Подтвердить лок после непрерывного слежения confirm_after_s.

        До подтверждения лок предварительный: эталон мог быть снят с неудачного кадра, и
        держаться за него — значит запереть дрона. После подтверждения включается строгий
        режим Ф4-11: только этот человек, иначе бессрочный поиск."""
        if self._identity_confirmed or not self._reid_active() or not self._identity.locked:
            return
        if self._lock_started is None:
            self._lock_started = self.get_clock().now()
            return
        held = (self.get_clock().now() - self._lock_started).nanoseconds * 1e-9
        if held >= self._reid_confirm_after_s:
            self._identity_confirmed = True
            self.get_logger().info(
                f"Лок ПОДТВЕРЖДЁН (track_id={self._locked_id}, вели {held:.1f} с): "
                f"дальше сопровождаю только этого человека.")

    def _embed_one(self, frame, bbox):
        crop = crop_bbox(frame, bbox, self._reid_crop_padding)
        emb = self._embedder.embed(crop)
        return emb

    def _maybe_reinforce(self, frame, detection):
        """Поддерживать эталон облика, пока трекер уверенно ведёт цель.

        Две работы:
        1) ДОЗАХВАТ. Если эталона ещё нет (на момент захвата кроп был пуст/мелок и
           эмбеддинг не посчитался) — записываем его при первой же годной возможности.
           Без этого дыра: трек ведётся, эталона нет, и после смерти трека сверять
           кандидатов не с чем — цель терялась бы безвозвратно.
        2) ПОДПИТКА. Человек поворачивается, вид со спины ≠ анфас; изредка добавляем
           свежий ракурс, иначе эталон устареет и гейт перестанет узнавать своего.

        Скважность `reid.refresh_every_n_frames` — следствие Ф4-15: эмбеддер делит GPU с
        YOLO и рендером Gazebo, каждый кадр его гонять нельзя. На дозахват скважность НЕ
        распространяется — эталон нужен как можно раньше."""
        if not self._reid_active():
            return
        bbox = detection[0]
        if (bbox[3] - bbox[1]) < self._reid_min_bbox_px:
            return   # слишком мелкий кроп — облик недостоверен, эталон только испортим

        need_seed = not self._identity.locked
        if not need_seed and (self._reid_refresh_every <= 0
                              or self._frame_idx % self._reid_refresh_every != 0):
            return

        emb = self._embed_one(frame, bbox)
        if emb is None:
            return
        if need_seed:
            self._identity.lock(emb)
            self.get_logger().info(
                f"Облик цели записан (дозахват, track_id={self._locked_id}) — ReID активен.")
        else:
            self._identity.reinforce(emb)

    def _relock(self, frame, tracked):
        """Найти среди кандидатов СВОЮ цель по эталону облика. None = своего нет.

        Троттлинг `reid.relock_check_interval_s`: пока цель не найдена, попытки идут
        постоянно, и без ограничения частоты эмбеддер выел бы GPU у детектора (R4)."""
        now = self.get_clock().now()
        if (self._last_relock_try is not None
                and (now - self._last_relock_try).nanoseconds * 1e-9 < self._reid_relock_period):
            return None
        self._last_relock_try = now

        # Слишком мелкие боксы отсеиваем ДО эмбеддинга: облик в них недостоверен, а
        # ложное совпадение на мелком кандидате — ровно та ошибка, которую запрещает Ф4-11.
        candidates = [d for d in tracked if (d[0][3] - d[0][1]) >= self._reid_min_bbox_px]
        if not candidates:
            return None

        crops = [crop_bbox(frame, d[0], self._reid_crop_padding) for d in candidates]
        embeddings = self._embedder.embed_batch(crops)
        if embeddings is None or len(embeddings) == 0:
            return None

        idx, score = self._identity.match(embeddings)
        self._last_score = score
        if self._reid_debug:
            # Баллы ПО КАЖДОМУ кандидату с его track_id и высотой bbox: только так видно,
            # различает ли порог своего и чужих, или все кандидаты лежат кучей.
            per = ', '.join(
                f'id{d[3]}(h={d[0][3] - d[0][1]}px)={s:.3f}'
                for d, s in zip(candidates, self._identity.scores(embeddings)))
            self.get_logger().info(
                f"ReID-сверка (порог {self._identity.similarity_threshold:.2f}): {per}",
                throttle_duration_sec=1.0)
        if idx is None:
            return None

        target = candidates[idx]
        self._locked_id = target[3]
        self.get_logger().info(
            f"ПЕРЕЗАХВАТ по облику: свой найден, track_id={self._locked_id}, "
            f"сходство {score:.3f} ≥ {self._identity.similarity_threshold}")
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

        # Статус идентичности (M2): видно, ведём ли мы СВОЕГО и насколько уверенно. Без
        # этого «дрон летит за человеком» неотличимо от «дрон летит за похожим человеком».
        # Текст ТОЛЬКО ASCII: cv2.putText не умеет кириллицу — русские строки выводились
        # как "??????" (найдено на прогоне 2026-07-18).
        if self._identity is not None:
            if not self._reid_active():
                status, tint = 'ReID: OFF', (0, 165, 255)
            elif target is not None:
                state = 'LOCKED' if self._identity_confirmed else 'LOCKING'
                status, tint = f'{state} id={self._locked_id}', (0, 200, 0)
            elif self._identity.locked:
                score = f'{self._last_score:.2f}' if self._last_score is not None else '-'
                status = (f'SEARCHING my target (best {score} < '
                          f'{self._identity.similarity_threshold:.2f})')
                tint = (0, 0, 255)
            else:
                status, tint = 'waiting for target', (200, 200, 200)
            cv2.putText(frame, status, (8, height - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, tint, 2, cv2.LINE_AA)
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
