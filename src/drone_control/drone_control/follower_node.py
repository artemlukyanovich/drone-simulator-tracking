"""follower_node — заглушка (Фаза 1).

Пока только heartbeat: подтверждает, что пакет собрался, нода стартует и launch её
поднимает. Реальная логика появится в Фазе 3: непрерывный offboard-цикл, который
читает /perception/target и телеметрию PX4 и каждый тик публикует setpoint
(P-регулятор). Важно: setpoint'ы должны идти непрерывно >2 Гц, иначе PX4 срывает
offboard — поэтому контроллер будет циклом, а не разовой командой (см. CLAUDE.md).
"""

import rclpy
from rclpy.node import Node


class FollowerNode(Node):
    """Нода-контроллер. В Фазе 1 не управляет дроном — только бьётся пульсом."""

    def __init__(self):
        super().__init__('follower_node')
        # Период heartbeat выносим в ros2-параметр, а не хардкодим (см. CLAUDE.md).
        period = self.declare_parameter('heartbeat_period', 2.0).value
        self._timer = self.create_timer(period, self._on_tick)
        self.get_logger().info('follower_node started (Phase 1 stub)')

    def _on_tick(self):
        self.get_logger().info('follower_node alive (stub, no control loop yet)')


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
