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
        # M3 (Ф4-5/Ф4-6): forward-канал по ЧЕСТНОЙ дистанции (метры), area — fallback.
        self._distance_target = float(self.declare_parameter('distance_target_m', 4.0).value)
        self._kp_distance = float(self.declare_parameter('kp_distance', 0.5).value)
        self._dist_db = float(self.declare_parameter('distance_deadband_m', 0.3).value)
        # M4 (Ф4-6): I и D канала дистанции. Дефолт 0 → чистый P, как в M3.
        self._ki_distance = float(self.declare_parameter('ki_distance', 0.0).value)
        self._kd_distance = float(self.declare_parameter('kd_distance', 0.0).value)
        self._i_limit_distance = float(self.declare_parameter('i_limit_distance', 2.0).value)
        # Guard кадрирования: цель у нижнего края (offset_y большой +, наземная цель при
        # наклоне вниз приближается → уходит вниз) → форсируем откат назад, чтобы не выпала.
        self._offset_y_backoff = float(self.declare_parameter('offset_y_backoff', 0.8).value)
        self._backoff_speed = float(self.declare_parameter('backoff_speed', 0.6).value)
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
        # M2 (Ф4-7/Ф4-11): поведение при потере цели.
        self._lost_hold_s = float(self.declare_parameter('lost_hold_s', 2.0).value)
        self._initial_hold_s = float(self.declare_parameter('initial_hold_s', 5.0).value)
        self._hold_after_loss_s = float(self.declare_parameter('hold_after_loss_s', 2.0).value)
        self._alt_tolerance = float(self.declare_parameter('follow_altitude_tolerance_m', 0.1).value)
        self._search_yaw_rate = float(self.declare_parameter('search_yaw_rate', 0.35).value)

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

        # --- FSM (Ф4-7) ---
        # TAKEOFF → набор высоты; TRACK → ведём цель; LOST → цель пропала, доворачиваем в
        # сторону, где её видели (короткая пауза против ложных срабатываний на моргании
        # детекции); SEARCH → медленное вращение на месте, БЕССРОЧНО (Ф4-11: ищем своего,
        # а не берём другого). Выход из SEARCH по таймауту в RTL — это M5, не здесь.
        self._state = 'TAKEOFF'
        self._lost_since = None        # когда цель пропала (для перехода LOST→SEARCH)
        self._last_offset_x = 0.0      # где видели цель последний раз → куда доворачивать
        self._altitude_reached = False  # защёлка «рабочая высота набрана» (см. _altitude_ready)
        self._ever_tracked = False      # была ли цель хоть раз в TRACK — влияет на выдержку
        # Состояние PID канала дистанции (M4). Сбрасывается при выходе из TRACK — см. _reset_pid.
        self._pid_integral = 0.0
        self._pid_prev_err = None
        self._pid_last_time = None

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
        ready = self._altitude_ready(altitude)
        self._update_state(ready)

        # Диагностика высоты: без неё «дрон летит низко» — это ощущение, а не число.
        # Троттлинг 2 с, чтобы не топить лог.
        self.get_logger().info(
            f"alt={altitude:.2f}/{self._target_alt:.2f} м  vz={vz:+.2f}  state={self._state}",
            throttle_duration_sec=2.0)

        if ready and self._target_valid():
            offset_x = self._target.offset_x
            offset_y = self._target.offset_y
            area_ratio = self._target.area_ratio
            distance = self._target.distance_m

            # Доворот к цели по offset_x (с мёртвой зоной).
            yawspeed = 0.0
            if abs(offset_x) > self._yaw_db:
                yawspeed = _clamp(self._yaw_sign * self._kp_yaw * offset_x, self._max_yaw)

            # Forward-канал (M3): держим ЧЕСТНУЮ дистанцию в метрах. Далеко (distance >
            # target) → вперёд; слишком близко (distance < target) → назад (естественный
            # откат). Нет честной оценки (distance==0) → fallback на area-прокси (Фаза 3).
            v_fwd = 0.0
            if distance > 0.0:
                dist_err = distance - self._distance_target
                # Мёртвая зона обрабатывается ВНУТРИ регулятора (см. _distance_pid): она
                # глушит P, но не интеграл — иначе возникает автоколебание.
                v_fwd = _clamp(self._fwd_sign * self._distance_pid(dist_err), self._max_fwd)
            else:
                area_err = self._area_target - area_ratio
                if abs(area_err) > self._area_db:
                    v_fwd = _clamp(self._fwd_sign * self._kp_fwd * area_err, self._max_fwd)

            # Guard кадрирования: цель у нижнего края → форсируем откат назад (в ту же
            # «от цели» сторону, что и большой dist_err), чтобы не выпала из кадра. Работает
            # и без честной дистанции (offset_y доступен всегда, пока detected).
            if offset_y > self._offset_y_backoff:
                v_fwd = -self._fwd_sign * self._backoff_speed

            self._last_offset_x = offset_x

            # body «вперёд» → NED по текущему heading.
            heading = self._pos.heading
            vx = v_fwd * math.cos(heading)
            vy = v_fwd * math.sin(heading)
            return vx, vy, vz, yawspeed

        # Цели нет. Вместо «просто висим» (Фаза 3) — активное поведение по состоянию FSM.
        # Горизонтальную скорость держим в нуле в обоих случаях: уходить с места вслепую
        # опасно и бессмысленно, ищем доворотом.
        if self._state == 'HOLD':
            # Замерли: курс НЕ меняем. Если цель просто зашла за другого человека, она
            # вернётся туда же — важно продолжать смотреть именно туда.
            return 0.0, 0.0, vz, 0.0
        if self._state == 'LOST':
            # Доворачиваем ТУДА, где цель видели последней: похоже, она ушла за край
            # кадра, и небольшого доворота хватает, чтобы вернуть её в поле зрения.
            direction = 1.0 if self._last_offset_x >= 0.0 else -1.0
            return 0.0, 0.0, vz, self._yaw_sign * direction * self._search_yaw_rate
        if self._state == 'SEARCH':
            # Бессрочное вращение на месте в ту же сторону — обзор 360°. Ф4-11: ждём
            # ИМЕННО своего; ReID не пропустит чужого, поэтому крутиться можно сколько нужно.
            direction = 1.0 if self._last_offset_x >= 0.0 else -1.0
            return 0.0, 0.0, vz, self._yaw_sign * direction * self._search_yaw_rate
        return 0.0, 0.0, vz, 0.0

    def _distance_pid(self, error):
        """PID канала дистанции (M4 / Ф4-6): ошибка [м] → скорость вперёд/назад [м/с].

        Зачем I. Чистый P физически не может свести ошибку к нулю за ДВИЖУЩЕЙСЯ целью: чтобы
        дрон ехал со скоростью цели, ему нужна ненулевая ошибка, кормящая член `kp·e`. При
        v_цели≈0.31 м/с и kp=0.5 равновесие приходится на e≈0.62 м — отсюда наблюдаемые
        4.6–4.8 м при уставке 4.0. Интеграл копит ошибку и берёт постоянную составляющую
        скорости на себя: когда e→0, накопитель НЕ обнуляется и продолжает выдавать нужные
        0.31 м/с. P после этого занимается только отклонениями.

        Зачем D. Реагирует на скорость изменения ошибки — тормозит заранее, гасит перелёт и
        раскачку, которую провоцирует I. Производная берётся от `distance_m`, уже сглаженной
        EMA в детекторе (`distance_ema_alpha`), поэтому отдельного фильтра здесь нет: если D
        начнёт шуметь, крутить надо ту EMA, а не заводить второй фильтр.

        При ki=kd=0 (дефолт) выражение вырождается в прежний P — поведение Фазы 3/M3."""
        now = self.get_clock().now()
        dt = (0.0 if self._pid_last_time is None
              else (now - self._pid_last_time).nanoseconds * 1e-9)
        self._pid_last_time = now

        # Первый тик после сброса (dt==0) или неправдоподобно большой разрыв (нода
        # подвисла/цель долго отсутствовала) — интеграл и производную пропускаем, иначе
        # один битый интервал даст скачок команды.
        # Мёртвая зона глушит P и D, но НЕ интеграл. Это принципиально: если обнулять
        # накопитель при входе в зону (так было сделано сначала), возникает автоколебание —
        # дрон подходит, попадает в зону, теряет накопленную скорость, встаёт, цель убегает,
        # и всё повторяется. На модели канала с запаздыванием: размах 0.34 м при сбросе
        # против 0.00 м, если интеграл оставить жить (проверено 2026-07-18).
        in_deadband = abs(error) <= self._dist_db

        if dt <= 0.0 or dt > 1.0:
            # Первый тик после сброса или битый интервал (нода подвисла): интеграл и
            # производную пропускаем, иначе один разрыв даст скачок команды.
            self._pid_prev_err = error
            return 0.0 if in_deadband else self._kp_distance * error

        self._pid_integral += error * dt
        # Anti-windup: зажим накопителя. Без него долгое отставание (цель убежала, дрон
        # упёрся в max_forward_speed) раздувает интеграл, и при возврате цели дрон рвёт
        # вперёд по инерции накопленного.
        self._pid_integral = _clamp(self._pid_integral, self._i_limit_distance)

        derivative = 0.0 if self._pid_prev_err is None else (error - self._pid_prev_err) / dt
        self._pid_prev_err = error

        if in_deadband:
            # Внутри зоны едем ТОЛЬКО на накопленном: это и есть постоянная скорость цели.
            # При ki=0 (дефолт) выход равен нулю — поведение M3 сохраняется в точности.
            return self._ki_distance * self._pid_integral

        return (self._kp_distance * error
                + self._ki_distance * self._pid_integral
                + self._kd_distance * derivative)

    def _reset_pid(self):
        """Обнулить состояние PID. Вызывается при выходе из TRACK и в мёртвой зоне.

        Критично для стыковки с FSM (7A): за время HOLD/LOST/SEARCH цели нет, ошибка не
        обновляется, и сохранённый интеграл к моменту перезахвата стал бы мусором — дрон
        дёрнулся бы вперёд сразу после того, как снова увидел человека."""
        self._pid_integral = 0.0
        self._pid_prev_err = None
        self._pid_last_time = None

    def _altitude_ready(self, altitude):
        """Разрешено ли слежение по высоте.

        Гейт работает ТОЛЬКО до первого захвата: дрон обязан сперва выйти на
        `target_altitude_m`, иначе он начинал преследование на середине набора и летел за
        человеком слишком низко. Дальше флаг ЗАЩЁЛКИВАЕТСЯ и высота больше не проверяется —
        иначе слежение прерывалось бы на каждом манёвре, где дрон немного просел.

        Отдельной выдержки на «устаканивание» не нужно: `vz = kp_altitude · ошибка`, поэтому
        попадание в узкий допуск само по себе означает, что вертикальная скорость уже мала.
        Жёсткий пол `min_follow_altitude_m` остаётся как страховка на все случаи."""
        if altitude <= self._min_follow_alt:
            return False
        if self._altitude_reached:
            return True
        if altitude >= self._target_alt - self._alt_tolerance:
            self._altitude_reached = True
            self.get_logger().info(
                f"Рабочая высота набрана ({altitude:.2f} из {self._target_alt:.2f} м) — "
                f"слежение разрешено.")
            return True
        return False

    def _update_state(self, ready):
        """Переходы FSM. Состояние меняется только здесь — один источник истины."""
        prev = self._state
        has_target = ready and self._target_valid()

        if not ready:
            self._state = 'TAKEOFF'
            self._lost_since = None
        elif has_target:
            self._state = 'TRACK'
            self._lost_since = None
            self._ever_tracked = True
        else:
            # Цели нет. Три фазы по нарастанию активности — каждая следующая рискованнее:
            #
            #   HOLD   — ЗАМЕРЕТЬ, курс не менять. Самая частая причина потери — цель
            #            пересеклась с другим человеком и на пару секунд скрылась. Доворот в
            #            этот момент отворачивает нас от неё же: она вернётся туда, куда мы
            #            уже не смотрим. Поэтому сперва просто ждём на месте.
            #   LOST   — доворот в сторону последнего появления: цель, похоже, ушла за край
            #            кадра, небольшой доворот вернёт её в поле зрения.
            #   SEARCH — вращение на месте, обзор 360°, бессрочно (Ф4-11).
            #
            # До первого захвата фаза LOST пропускается: доворачивать некуда, цели ещё не
            # было. Дрон просто висит `initial_hold_s` и осматривается — иначе он успевал
            # сделать полный оборот прежде, чем заметить стоящего перед ним человека.
            now = self.get_clock().now()
            if self._lost_since is None:
                self._lost_since = now
            lost_for = (now - self._lost_since).nanoseconds * 1e-9

            if not self._ever_tracked:
                self._state = 'HOLD' if lost_for < self._initial_hold_s else 'SEARCH'
            elif lost_for < self._hold_after_loss_s:
                self._state = 'HOLD'
            elif lost_for < self._hold_after_loss_s + self._lost_hold_s:
                self._state = 'LOST'
            else:
                self._state = 'SEARCH'

        if self._state != 'TRACK':
            # Вне слежения ошибка дистанции не обновляется — накопленный интеграл к моменту
            # перезахвата стал бы мусором и дёрнул бы дрона вперёд (стык M4 с FSM 7A).
            self._reset_pid()

        if self._state != prev:
            self.get_logger().info(f"FSM: {prev} → {self._state}")

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
