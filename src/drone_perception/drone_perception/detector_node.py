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
from sensor_msgs.msg import Image

from drone_interfaces.msg import Target

from drone_perception.detector import ObjectDetector


class DetectorNode(Node):
    """Детектор цели: кадр → bbox → нормализованное смещение от центра."""

    def __init__(self):
        super().__init__('detector_node')

        # --- Параметры (значения по умолчанию переопределяются из yaml) ---
        self._image_topic = self.declare_parameter('image_topic', '/camera/image').value
        self._target_topic = self.declare_parameter('target_topic', '/perception/target').value
        self._annotated_topic = self.declare_parameter('annotated_topic', '/perception/image').value
        self._publish_annotated = self.declare_parameter('publish_annotated', True).value
        model_path = self.declare_parameter('model_path', 'yolov8n.pt').value
        confidence = self.declare_parameter('confidence_threshold', 0.5).value
        device = self.declare_parameter('device', 'cuda').value
        # Какие классы считаем целью (COCO-имена). Пусто → любой класс.
        self._target_classes = list(self.declare_parameter('target_classes', ['person']).value)
        # Критерий выбора одной цели из нескольких: "largest" | "closest".
        self._selection = self.declare_parameter('selection', 'largest').value

        self._bridge = CvBridge()
        self.get_logger().info(
            f"Загружаю YOLO: model={model_path} device={device} conf={confidence}")
        self._detector = ObjectDetector(
            model_path=model_path,
            confidence_threshold=confidence,
            device=device,
        )

        # Камера — sensor data (best-effort), поэтому такой же QoS на подписке.
        self._sub = self.create_subscription(
            Image, self._image_topic, self._on_image, qos_profile_sensor_data)
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

        best = self._select(detections, width, height)
        if best is not None:
            (x1, y1, x2, y2), class_name, conf = best
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            target_msg.detected = True
            target_msg.offset_x = float((cx - width / 2.0) / (width / 2.0))
            target_msg.offset_y = float((cy - height / 2.0) / (height / 2.0))
            target_msg.area_ratio = float(((x2 - x1) * (y2 - y1)) / (width * height))
        else:
            target_msg.detected = False
            target_msg.offset_x = 0.0
            target_msg.offset_y = 0.0
            target_msg.area_ratio = 0.0

        self._target_pub.publish(target_msg)

        if self._annotated_pub is not None:
            self._publish_overlay(frame, best, width, height, msg.header)

    def _select(self, detections, width, height):
        """Выбрать одну цель из нескольких. largest = крупнейший bbox,
        closest = ближайший к центру кадра."""
        if not detections:
            return None
        if self._selection == 'closest':
            fx, fy = width / 2.0, height / 2.0

            def dist2(d):
                (x1, y1, x2, y2), _, _ = d
                return ((x1 + x2) / 2.0 - fx) ** 2 + ((y1 + y2) / 2.0 - fy) ** 2

            return min(detections, key=dist2)
        # default: largest
        return max(detections, key=lambda d: (d[0][2] - d[0][0]) * (d[0][3] - d[0][1]))

    def _publish_overlay(self, frame, best, width, height, header):
        # Центр кадра (цель наведения) — зелёный крест.
        cx0, cy0 = width // 2, height // 2
        cv2.drawMarker(frame, (cx0, cy0), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
        if best is not None:
            (x1, y1, x2, y2), class_name, conf = best
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.line(frame, (cx0, cy0), ((x1 + x2) // 2, (y1 + y2) // 2), (0, 0, 255), 1)
            cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
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
