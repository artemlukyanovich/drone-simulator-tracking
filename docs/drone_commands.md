# Шпаргалка: базовые команды проверки управления дроном

> Назначение: быстрый справочник команд для проверки корректности SITL/управления —
> армится ли дрон, в каком режиме, идёт ли телеметрия, что командует контроллер. Не
> привязан к конкретной фазе. Окружение терминалов — как в `docs/phase2_setup.md`
> («Подготовка»: три `source`). Стоп симуляции между прогонами — `scripts/stop_sim.sh`.

## 1. Консоль PX4 (`pxh>`, Терминал 1 — где запущен SITL)

Это команды самого автопилота, печатаются в приглашении `pxh>`.

| Команда | Что делает |
|---|---|
| `commander status` | арминг (ARMED/DISARMED), текущий режим полёта, состояние failsafe |
| `commander check` | прогнать preflight-проверки и **перечислить, что не прошло** (главное при `Arming denied`) |
| `commander arm` | взвести (моторы on, разрешить полёт); `-f` — форс, пропустить проверки |
| `commander disarm` | разоружить (моторы off); `-f` — форс, в т.ч. **в воздухе** (аварийно) |
| `commander takeoff` | авто-взлёт (режим AUTO.TAKEOFF — НЕ offboard, наши setpoint'ы игнорируются) |
| `commander land` | авто-посадка (режим AUTO.LAND) |
| `commander mode <режим>` | сменить режим: `offboard`, `posctl`, `altctl`, `hold`, `auto:rtl`, `auto:loiter`, `manual`, … |
| `commander mode offboard` | войти в offboard (сработает только если поток внешних setpoint'ов уже идёт) |
| `listener <topic> [N]` | напечатать N сэмплов uORB-топика прямо в PX4, напр. `listener vehicle_local_position 1`, `listener vehicle_status`, `listener battery_status` |
| `shutdown` | корректно погасить PX4 (в обычном режиме — и связанный Gazebo) |

⚠️ Пока работает наш `follower_node`, он каждую секунду заново армит и удерживает offboard —
ручные `arm`/`disarm`/`land`/`mode` будут перебиваться. Для ручного управления **сперва
останови контроллер** (Ctrl-C в его терминале).

## 2. ROS2-сторона (терминал с засорсенным окружением)

Проверки моста PX4↔ROS2, перцепции и команд контроллера.

| Команда | Что проверяет |
|---|---|
| `ros2 topic list \| grep '^/fmu/'` | мост uXRCE-DDS жив (десятки `/fmu/*` топиков) |
| `ros2 topic hz /fmu/out/vehicle_local_position` | телеметрия идёт (~100 Гц) |
| `ros2 topic echo --once /fmu/out/vehicle_local_position` | позиция `x/y/z` (NED, z<0=вверх), `heading` |
| `ros2 topic echo --once /fmu/out/vehicle_status` | `arming_state`, `nav_state` (коды — §3) |
| `ros2 topic hz /camera/image` | камера идёт (~30 Гц) |
| `ros2 topic echo /perception/target` | выход детектора: `detected`, `offset_x/y`, `area_ratio` |
| `ros2 topic hz /perception/image` | поток аннотированных кадров (bbox) |
| `ros2 run rqt_image_view rqt_image_view /perception/image` | **глазами**: что видит дрон + рамка |
| `ros2 topic echo /fmu/in/trajectory_setpoint` | что **командует контроллер** (velocity NED, yawspeed) |
| `ros2 topic echo /fmu/in/vehicle_command` | команды arm/offboard от контроллера |
| `ros2 node list` / `ros2 node info /follower_node` | какие ноды живы, их топики/параметры |
| `ros2 param get /follower_node kp_yaw` | прочитать параметр на лету |

## 3. Расшифровка кодов телеметрии

`vehicle_status` (`/fmu/out/vehicle_status`):

- **`arming_state`**: `1` = STANDBY (disarmed), `2` = **ARMED**.
- **`nav_state`** (режим): `4` = AUTO.LOITER, `14` = **OFFBOARD**, `17` = AUTO.TAKEOFF,
  `18` = AUTO.LAND, `20` = AUTO.RTL. (Полный список — в `px4_msgs/msg/VehicleStatus`.)

`vehicle_local_position` (`/fmu/out/vehicle_local_position`): координаты в **NED**
(`x`=север, `y`=восток, `z`=вниз → **высота = −z**); `heading` — рыскание, рад, 0=север.

## 4. Типовые сценарии проверки

**«Дрон не армится» (`Arming denied`)**
1. `pxh> commander check` — увидеть конкретный неуспешный чек (GPS/EKF/магнитометр…).
2. Дать sim постоять ~15–20 с (EKF/GPS сходятся не сразу).
3. Убедиться, что не висят процессы прошлого прогона: на хосте
   `pgrep -af "gz sim|bin/px4|MicroXRCEAgent|parameter_bridge"` → пусто (иначе `scripts/stop_sim.sh`).

**Проверить offboard вручную (без контроллера)**
1. Запустить стрим setpoint'ов (наш `follower_node` или `ros2 topic pub`).
2. `pxh> commander mode offboard` → `pxh> commander arm`.
3. `pxh> commander status` — должно стать `OFFBOARD` + `ARMED`.

**Проверить, что командует контроллер**
- `ros2 topic echo /fmu/in/trajectory_setpoint` — поля `velocity` (NED) и `yawspeed`
  должны меняться в ответ на `/perception/target`.

**Аккуратная остановка полёта**
1. Ctrl-C терминал с контроллером (иначе перебьёт следующие команды).
2. `pxh> commander land` (или `commander disarm`).
3. `pxh> shutdown` → `scripts/stop_sim.sh`.
