"""follower_node — offboard-контроллер слежения за целью (Фаза 3, инкремент 3).

Непрерывный offboard-цикл: каждый тик публикует OffboardControlMode +
TrajectorySetpoint (даже без цели — hover), иначе PX4 срывает offboard (>2 Гц, §5/§7).
Порядок входа (§5): стримим setpoint'ы → DO_SET_MODE offboard → arm.

P-регулятор (Ф3-3, body-frame velocity):
  offset_x (цель право/лево центра) → yaw-rate (доворот к цели);
  area_ratio (размер bbox = дистанция) → скорость вперёд/назад;
  высота удерживается P-регулятором по target_altitude (он же делает взлёт).

Система координат (§Ф3-5): команду «вперёд» считаем в body, затем поворачиваем в NED
по текущему heading (TrajectorySetpoint.velocity — в NED). Знаки доворота/хода вынесены
в параметры yaw_sign/forward_sign — если в симуляторе крутит/едет не туда, переключаем
в конфиге без правки кода (frame-знаки — известный риск, §9).

Все настройки — ros2-параметры (configs/control/follower.yaml). Тест без детектора:
публикуем фейковый /perception/target через `ros2 topic pub` (инкремент 3).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

from drone_interfaces.msg import Target

NAN = float('nan')


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class FollowerNode(Node):
    """Offboard-цикл + P-регулятор слежения за целью."""

    def __init__(self):
        super().__init__('follower_node')

        # --- Параметры (значения из configs/control/follower.yaml) ---
        self._rate = float(self.declare_parameter('loop_rate_hz', 20.0).value)
        target_topic = self.declare_parameter('target_topic', '/perception/target').value
        self._target_alt = float(self.declare_parameter('target_altitude_m', 2.5).value)
        self._min_follow_alt = float(self.declare_parameter('min_follow_altitude_m', 1.0).value)
        self._arm_after_s = float(self.declare_parameter('arm_after_s', 2.0).value)
        self._target_timeout = float(self.declare_parameter('target_timeout_s', 0.5).value)
        # P-коэффициенты.
        self._kp_yaw = float(self.declare_parameter('kp_yaw', 1.2).value)
        self._kp_fwd = float(self.declare_parameter('kp_forward', 3.0).value)
        self._kp_alt = float(self.declare_parameter('kp_altitude', 1.0).value)
        self._area_target = float(self.declare_parameter('area_target', 0.15).value)
        # Лимиты скоростей.
        self._max_yaw = float(self.declare_parameter('max_yaw_rate', 0.6).value)
        self._max_fwd = float(self.declare_parameter('max_forward_speed', 1.5).value)
        self._max_vz = float(self.declare_parameter('max_vertical_speed', 1.0).value)
        # Мёртвые зоны (не дёргаться на мелких ошибках).
        self._yaw_db = float(self.declare_parameter('yaw_deadband', 0.05).value)
        self._area_db = float(self.declare_parameter('area_deadband', 0.02).value)
        # Знаки (frame-неоднозначность — крутит/едет не туда → поменять в конфиге).
        self._yaw_sign = float(self.declare_parameter('yaw_sign', 1.0).value)
        self._fwd_sign = float(self.declare_parameter('forward_sign', 1.0).value)

        # QoS PX4 (как в офиц. примере px4_ros_com): best-effort + transient_local.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self._offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self._setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self._command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._on_local_position, px4_qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self._on_status, px4_qos)
        self.create_subscription(Target, target_topic, self._on_target, 10)

        # Состояние.
        self._pos = None            # последняя VehicleLocalPosition
        self._status = None         # последняя VehicleStatus (arming/nav state)
        self._target = None         # последняя Target
        self._target_time = None    # время приёма цели (для staleness)
        self._tick = 0
        self._engaged_logged = False
        self._last_engage_time = None

        self._timer = self.create_timer(1.0 / self._rate, self._on_tick)
        self.get_logger().info(
            f"follower_node готов: target={target_topic}, alt={self._target_alt} м, "
            f"loop={self._rate} Гц. Стримлю setpoint'ы, затем offboard+arm "
            f"через {self._arm_after_s} с.")

    # --- Колбэки подписок ---
    def _on_local_position(self, msg: VehicleLocalPosition):
        self._pos = msg

    def _on_status(self, msg: VehicleStatus):
        self._status = msg

    def _on_target(self, msg: Target):
        self._target = msg
        self._target_time = self.get_clock().now()

    # --- Основной offboard-цикл ---
    def _on_tick(self):
        self._publish_offboard_mode()

        vx, vy, vz, yawspeed = self._compute_setpoint()
        self._publish_setpoint(vx, vy, vz, yawspeed)

        # После прогрева стрима — добиваемся offboard + arm (порядок §5). ПОВТОРЯЕМ,
        # пока PX4 реально не перейдёт в ARMED+OFFBOARD: первая попытка часто отвергается,
        # если preflight ещё не прошёл (GPS/EKF не готовы первые секунды).
        self._tick += 1
        if self._tick >= int(self._arm_after_s * self._rate):
            self._ensure_offboard_and_armed()

    def _compute_setpoint(self):
        """Вернуть (vx, vy, vz, yawspeed) в NED. Без позиции/телеметрии — hover."""
        if self._pos is None:
            return 0.0, 0.0, 0.0, 0.0

        # Высота: P-регулятор по target (он же выполняет взлёт). NED: z<0 = вверх.
        z_target = -self._target_alt
        vz = _clamp(self._kp_alt * (z_target - self._pos.z), self._max_vz)

        altitude = -self._pos.z
        airborne = altitude > self._min_follow_alt
        if airborne and self._target_valid():
            offset_x = self._target.offset_x
            area_ratio = self._target.area_ratio

            # Доворот к цели по offset_x (с мёртвой зоной).
            yawspeed = 0.0
            if abs(offset_x) > self._yaw_db:
                yawspeed = _clamp(self._yaw_sign * self._kp_yaw * offset_x, self._max_yaw)

            # Дистанция по размеру bbox: мелкий (далеко) → вперёд; крупный → назад.
            v_fwd = 0.0
            area_err = self._area_target - area_ratio
            if abs(area_err) > self._area_db:
                v_fwd = _clamp(self._fwd_sign * self._kp_fwd * area_err, self._max_fwd)

            # body «вперёд» → NED по текущему heading.
            heading = self._pos.heading
            vx = v_fwd * math.cos(heading)
            vy = v_fwd * math.sin(heading)
            return vx, vy, vz, yawspeed

        # Цель потеряна/на земле — горизонталь держим в нуле, высоту регулируем.
        return 0.0, 0.0, vz, 0.0

    def _target_valid(self):
        if self._target is None or self._target_time is None or not self._target.detected:
            return False
        age = (self.get_clock().now() - self._target_time).nanoseconds * 1e-9
        return age <= self._target_timeout

    # --- Публикация в PX4 ---
    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self._now_us()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self._offboard_pub.publish(msg)

    def _publish_setpoint(self, vx, vy, vz, yawspeed):
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        msg.position = [NAN, NAN, NAN]       # позицию не задаём — чистое velocity-управление
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.acceleration = [NAN, NAN, NAN]
        msg.jerk = [NAN, NAN, NAN]
        msg.yaw = NAN                         # абсолютный yaw не задаём — рулим скоростью
        msg.yawspeed = float(yawspeed)
        self._setpoint_pub.publish(msg)

    def _ensure_offboard_and_armed(self):
        """Повторять offboard+arm раз в ~1 с, пока PX4 не подтвердит ARMED+OFFBOARD."""
        armed = (self._status is not None
                 and self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        offboard = (self._status is not None
                    and self._status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        if armed and offboard:
            if not self._engaged_logged:
                self.get_logger().info("PX4 в OFFBOARD и ARMED — слежение активно.")
                self._engaged_logged = True
            return

        now = self.get_clock().now()
        if (self._last_engage_time is not None
                and (now - self._last_engage_time).nanoseconds * 1e-9 < 1.0):
            return  # не чаще раза в секунду
        self._last_engage_time = now

        # Offboard PX4 примет, только если поток setpoint'ов уже идёт (он идёт, §5).
        if not offboard:
            self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        if not armed:
            self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info(
            f"Добиваюсь режима: armed={armed}, offboard={offboard} "
            f"(шлю offboard+arm)…")

    def _send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._command_pub.publish(msg)

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


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
