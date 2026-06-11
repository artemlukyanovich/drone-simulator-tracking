"""detector_node — заглушка (Фаза 1).

Пока только heartbeat: подтверждает, что пакет собрался, нода стартует и launch её
поднимает. Реальная логика (подписка на /camera/image, YOLO, публикация bbox и
смещения от центра в /perception/target) появится в Фазе 3.
"""

import rclpy
from rclpy.node import Node


class DetectorNode(Node):
    """Нода-детектор. В Фазе 1 не делает перцепции — только бьётся пульсом."""

    def __init__(self):
        super().__init__('detector_node')
        # Период heartbeat выносим в ros2-параметр, а не хардкодим (см. CLAUDE.md).
        period = self.declare_parameter('heartbeat_period', 2.0).value
        self._timer = self.create_timer(period, self._on_tick)
        self.get_logger().info('detector_node started (Phase 1 stub)')

    def _on_tick(self):
        self.get_logger().info('detector_node alive (stub, no perception yet)')


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
