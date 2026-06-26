# Фаза 3 — MVP «вижу → двигаюсь» (план реализации и решения)

> Назначение: подробно зафиксировать **что и как** реализуется в Фазе 3, принятые
> решения с обоснованием, порядок инкрементов (каждый — свой go/no-go), способы проверки
> и визуализации, ключевые ловушки. По мере реализации этот файл дополняется фактами
> (как `docs/phase2_setup.md`), а статусные строки помечаются ✅.
>
> Источник истины по архитектуре — `docs/project_plan.md` (§5, §8.9–8.10). Здесь — детали
> именно Фазы 3. Версии стека и sim-сторона — Фаза 2 (`docs/phase2_setup.md`), они не меняются.
>
> Статус: 🚧 **в реализации** (2026-06-26). Инкременты 0 ✅ (`drone_interfaces`+`Target.msg`),
> 1 ✅ (свой мир `follow_target.sdf` + standalone-запуск, камера с человеком подтверждена,
> §11). Инкремент 2 🚧 — `detector_node` (YOLO → `/perception/target` + `/perception/image`):
> собран, YOLO детектит модель оффлайн, осталось подтвердить live на sim (§12). `follower_node`
> пока heartbeat-заглушка (Фаза 1).

---

## 1. Цель Фазы 3

Замкнуть петлю **«вижу → решаю → двигаюсь»**: дрон детектирует цель своей камерой
(`/camera/image`) и физически следует за ней в Gazebo, удерживая её в кадре и на заданной
дистанции. Это первый момент, когда перцепция реально управляет аппаратом.

```
Gazebo камера → /camera/image (rgb8, 30 Гц)
   → detector_node (YOLO, выбор одной цели) → /perception/target (нормализованные offset+размер)
                                            → /perception/image  (кадр с нарисованным bbox — для глаз)
   → follower_node (offboard P-цикл) → /fmu/in/offboard_control_mode
                                      + /fmu/in/trajectory_setpoint
                                      + /fmu/in/vehicle_command (arm + offboard)
   → PX4 SITL → моторы в Gazebo
```

## 2. Принятые решения Фазы 3 (с обоснованием)

| # | Решение | Почему так |
|---|---------|-----------|
| Ф3-1 | **Цель — движущийся человек** (Gazebo `actor` с траекторией ходьбы). Класс COCO `person`. | Близко к реальному сценарию «follow-me». Детектор из project_1 заточен под `person` (`allowed_classes: ["person"]`). Де-риск: сперва **статично** проверяем, что YOLO вообще ловит меш actor'а, затем включаем ходьбу. |
| Ф3-2 | **Свой интерфейс `drone_interfaces/Target.msg`**, не `vision_msgs`. | Для follow-me с одной целью сообщение несёт ровно управляющую величину (нормализованный offset + размер bbox), не зависит от разрешения кадра и развязывает детектор/контроллер от пикселей. `vision_msgs/Detection2DArray` оправдан, когда один перцепционный стек кормит много разных потребителей — у нас один потребитель с конкретной нуждой, и он не должен парсить массив боксов и сам считать геометрию. Бонус: кастомные интерфейсы были отложены с Фазы 1 — закрываем этот навык ROS2. |
| Ф3-3 | **Степени свободы MVP: yaw + вперёд/назад, высота фиксирована.** | Минимальный надёжный offboard-цикл (план §8.10). `offset_x` → разворот к цели; `area_ratio` (размер bbox) → дистанция. Боковой снос и подстройку высоты по `offset_y` оставляем на потом (Фаза 4 / расширение), чтобы не настраивать сразу много осей P-регулятора. |
| Ф3-4 | **Детектор переносим как логику из project_1**, без трекинга/ReID/counter/CLIP. | Дрону для MVP нужна только детекция одной цели за кадр. Трекинг и (опц.) ReID — Фаза 4 (план §8.12). Дивергенция от project_1 ожидаема и здорова (решение Р3). |
| Ф3-5 | **Координатная рамка setpoint'ов фиксируется явно: body-frame velocity.** | Перевод «смещение bbox → команда» естественнее в body (вперёд/разворот относительно дрона), чем в NED. Фиксируем явно во избежание путаницы знаков (ловушка §9 плана). |

## 3. Новый пакет: `drone_interfaces`

Кастомные ROS2-сообщения **нельзя** держать в `ament_python`-пакете — генерация типов
требует `rosidl`, поэтому заводим отдельный **`ament_cmake`**-пакет (структурное исключение
из конвенции «всё python», §7 плана — это нормально, как двойная вложенность ament_python).

```
src/drone_interfaces/
├── CMakeLists.txt          # rosidl_generate_interfaces(...)
├── package.xml             # depends: std_msgs; build/exec: rosidl_default_*
└── msg/
    └── Target.msg
```

`Target.msg` (минимальный, в нормализованных координатах):

```
std_msgs/Header header     # стамп кадра — для контроля «свежести» цели (staleness)
bool    detected           # есть ли цель в кадре прямо сейчас
float32 offset_x           # [-1..1] смещение центра bbox от центра кадра по X (право > 0)
float32 offset_y           # [-1..1] по Y (вниз > 0)
float32 area_ratio         # [0..1] доля площади кадра под bbox — прокси дистанции
```

`header.stamp` нужен контроллеру, чтобы понимать «детектор молчит → перейти в hover», а не
слепо рулить по устаревшему `offset` (защита при потере цели / зависании детектора).

После добавления пакета `colcon build` собирает и его; контроллер/детектор объявляют
`<depend>drone_interfaces</depend>` в своих `package.xml`.

## 4. Детектор (`drone_perception/detector_node`)

Внутреннюю логику берём из project_1 `src/detector.py` — класс `ObjectDetector` (YOLO,
выбор backend `.pt`/`.onnx`/`.engine`, порог уверенности) переносится почти 1:1. **Не**
переносим `tracker.py`, `reid.py`, `counter.py`, CLIP-эмбеддинги — это Фаза 4.

Нода делает:
1. Подписка на `/camera/image` (`sensor_msgs/Image`, `rgb8`, 640×480, 30 Гц).
2. `cv_bridge`: ROS Image → numpy-массив; конвертация **RGB → BGR** (см. §6).
3. YOLO-инференс, фильтр по классу `person` и порогу (из `configs/perception/`).
4. **Выбор одной цели** из нескольких детекций — новая маленькая логика (project_1 считал
   все объекты, дрону нужна одна): по умолчанию крупнейший bbox; вариант — ближайший к
   центру кадра. Критерий вынести в конфиг.
5. Расчёт нормализованных `offset_x/offset_y/area_ratio` (см. §7) и публикация
   `/perception/target`.
6. (Для глаз) отрисовка bbox/подписи на кадре и публикация `/perception/image`.

Конфиг — `configs/perception/` (порог, класс, путь к весам `yolov8n.pt`, критерий выбора
цели). Веса модели — gitignored (как в project_1). Зависимости (`ultralytics`/`torch`,
`cv_bridge`) — в venv.

## 5. Контроллер (`drone_control/follower_node`) — критичная часть

Offboard-управление PX4 чувствительно к деталям (ловушки §7, §9 плана):

- **Это постоянный цикл, а не разовая команда.** Таймер (~20–50 Гц) каждый тик публикует
  `OffboardControlMode` + `TrajectorySetpoint`, даже когда цель потеряна (тогда — hover /
  нулевая скорость). Если поток setpoint'ов прерывается >0.5 с — PX4 срывает offboard.
- **Порядок входа:** сначала несколько секунд **стримим** setpoint'ы → командой
  `VehicleCommand` переключаем в offboard → **arm**. Не наоборот.
- **Координаты:** body-frame velocity (решение Ф3-5). Маппинг:
  - `offset_x` (цель левее/правее центра) → **yaw-rate** (доворот к цели);
  - `area_ratio` мал (цель далеко/мелкая) → скорость **вперёд**; велик → **назад**;
  - высота держится постоянной (z-setpoint фиксирован).
- **P-регулятор** (коэффициенты и лимиты скоростей — в `configs/control/`, без магических
  чисел). Переход к PID — Фаза 4 (план §8.11).
- **Свежесть цели:** если `header.stamp` старее порога или `detected=false` — hover
  (безопасное поведение), не рулить по устаревшему offset.

Подписки: `/perception/target` (+ телеметрия `/fmu/out/vehicle_local_position` для статуса).
Публикации: `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`,
`/fmu/in/vehicle_command`.

## 6. Почему RGB/BGR и зачем `cv_bridge`

**Две системы исторически условились о разном порядке байтов цвета:**

- **OpenCV** (и весь CV-код project_1: `detector.py`, рендер, видео) работает с кадрами в
  порядке каналов **BGR** (Blue-Green-Red). Это историческое наследие OpenCV — все его
  функции (`imread`, `imshow`, отрисовка) предполагают BGR.
- **ROS2** (`sensor_msgs/Image`) и Gazebo-камера отдают кадр в **RGB** (`encoding: rgb8` —
  именно это мы видели в Фазе 2). ROS — «общий язык» между разными нодами, поэтому порядок
  фиксируется в поле `encoding` сообщения, а не подразумевается.

Если скормить RGB-кадр коду, который ждёт BGR (или наоборот), каналы R и B меняются местами:
картинка приобретает синюшный/оранжевый оттенок. YOLO от этого деградирует — модель обучена
на корректных цветах, и «синие лица» детектируются хуже. Поэтому конвертация обязательна.

**`cv_bridge`** — это и есть мост между двумя мирами: пакет ROS2, который превращает
`sensor_msgs/Image` ↔ массив OpenCV (numpy), **с учётом encoding'а**. Вместо ручного
разбора байтов сообщения мы пишем:

```python
from cv_bridge import CvBridge
bridge = CvBridge()
frame_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  # ROS rgb8 → OpenCV BGR
```

`cv_bridge` сам читает, что пришёл `rgb8`, и отдаёт BGR-массив, который ожидает YOLO/OpenCV.
То есть он решает обе задачи разом: (1) распаковку ROS-сообщения в numpy и (2) корректную
перестановку каналов. Это стандартный способ соединять ROS-камеру и OpenCV-код.

## 7. Что такое `offset` в этом контексте

`offset` — **на сколько цель смещена от центра кадра**, то есть ошибка наведения, которую
контроллер обнуляет. Это связующее звено «вижу → двигаюсь».

Камера видит кадр 640×480. У детекции есть центр bbox в пикселях `(cx, cy)`. Центр кадра —
`(320, 240)`. «Сырое» смещение — `cx - 320` по X и `cy - 240` по Y. Но контроллеру неудобно
работать в пикселях (зависит от разрешения), поэтому **нормализуем в диапазон [-1..1]**:

```
offset_x = (cx - W/2) / (W/2)     # -1 = цель у левого края, 0 = по центру, +1 = у правого
offset_y = (cy - H/2) / (H/2)     # -1 = верх кадра,        0 = по центру, +1 = низ
```

Смысл для управления:
- `offset_x > 0` → цель **правее** центра → дрону нужно **довернуть вправо** (yaw), чтобы
  вернуть её в центр. `offset_x ≈ 0` → цель по центру, доворот не нужен.
- В MVP `offset_y` не используем (высота фиксирована, см. Ф3-3); он зарезервирован под
  будущую подстройку высоты.

Отдельно `area_ratio` (доля площади кадра под bbox) — **прокси дистанции**: чем ближе цель,
тем крупнее bbox. Контроллер держит её около целевого значения (вперёд, если объект мелкий
= далеко; назад, если крупный = близко).

Нормализация важна тем, что P-коэффициенты в `configs/control/` становятся независимыми от
разрешения камеры: поменяем камеру на 1280×720 — `offset` всё так же в [-1..1], настройки не
плывут.

## 8. План инкрементов (каждый — самостоятельный go/no-go)

| # | Что делаем | Go/no-go |
|---|---|---|
| **0** ✅ | Пакет `drone_interfaces` (`ament_cmake`) + `Target.msg`. `colcon build`. | `ros2 interface show drone_interfaces/msg/Target` печатает поля |
| **1** ✅ | Свой world-SDF с человеком (сначала **статично**), standalone-запуск (см. §11). | мир, человек, дрон в Gazebo; человек виден в `/camera/image` ✅ (2026-06-26) |
| **2** 🚧 | `detector_node`: порт `ObjectDetector`, RGB→BGR, YOLO, выбор одной цели, публикация `/perception/target` (+ `/perception/image`). | YOLO детектит модель оффлайн ✅; live `/perception/target` + bbox в `rqt_image_view` — подтвердить на sim |
| **3** | `follower_node`: offboard-цикл (стрим→offboard→arm), P-регулятор, конфиг `configs/control/`. Тест с **фейковым** target через `ros2 topic pub`. | дрон армится, входит в offboard, реагирует на фейковый offset; offboard не срывается |
| **4** | Полный пайплайн в `tracking_demo.launch.py` (детектор+контроллер+конфиги). Сперва статичная цель, затем **включить ходьбу** actor'а. | дрон взлетает, доворачивается к человеку, держит дистанцию; при движении цели — следует |

Порядок неслучаен: без объекта в мире (инкр. 1) детектор тестировать не на чем; контроллер
(инкр. 3) безопаснее обкатать на фейковом target до подключения живого детектора.

## 9. Ключевые ловушки и открытые детали Фазы 3

- **Детект меша человека** — ✅ де-рискнуто (2026-06-26): `yolov8n` детектит Fuel-модель
  `Standing person` как `person` с conf **0.81–0.92** на 4/5 ракурсов превью (промах лишь на
  нетипичном ракурсе). Камера смотрит вперёд на уровне роста — «хорошие» ракурсы. Для
  ходячего actor'а позже стоит перепроверить (другой меш/анимация).
- **Как прокинуть свой мир в PX4** — ✅ решено (standalone, §11): сами поднимаем Gazebo с
  нашим SDF, PX4 цепляется (`PX4_GZ_STANDALONE`). `<world>` обязан называться `default`
  (в standalone PX4 не определяет имя работающего мира, `PX4_GZ_WORLD` остаётся `default`).
- **RGB/BGR** при `cv_bridge` (§6) — забыть конверсию = деградация YOLO.
- **Ориентация камеры** `x500_mono_cam` — ✅ подтверждено: монтаж `.12 .03 .242 0 0 0`,
  углы нулевые → камера смотрит **вперёд (+X body) горизонтально**, FOV≈100°. Цель ставим
  впереди по +X.
- **Offboard >2 Гц и порядок arm/offboard** (§5) — главные грабли управления PX4.
- **Кастомные msg → `ament_cmake`** (§3), не `ament_python` — это не ошибка структуры.

## 10. Проверка и визуализация (как «пощупать» результат)

Окружение терминалов — как в Фазе 2 (`docs/phase2_setup.md`, раздел «Подготовка»: три
`source`). Sim поднимается так же: Терминал 1 — `scripts/run_px4_sitl.sh` (теперь с нашим
миром), Терминал 2 — `ros2 launch drone_simulator sim.launch.py` (агент + камера-мост),
плюс пайплайн Фазы 3 через `ros2 launch drone_bringup tracking_demo.launch.py`.

| Видишь | Значит работает |
|---|---|
| Человек в окне Gazebo и в `/camera/image` | инкр. 1: мир с целью |
| `ros2 topic echo /perception/target` → живой `offset_x`; bbox в `rqt_image_view /perception/image` | инкр. 2: детектор |
| Дрон армится и входит в offboard на фейковом target (`ros2 topic pub`) | инкр. 3: контроллер |
| Дрон в окне Gazebo доворачивается к человеку и держит дистанцию; следует при ходьбе | инкр. 4: **гейт Фазы 3** |

Инструменты наблюдения:
- **Окно Gazebo (GUI)** — дрон физически следует за целью (главное подтверждение).
- **`rqt_image_view /perception/image`** — что «видит» дрон, с рамкой и подписью.
- **`rqt_plot` по `/perception/target` `offset_x`** — видно, как P-регулятор сводит ошибку
  наведения к нулю во времени.
- Запись демо-видео и телеметрии в `outputs/` — формально Фаза 4, но удобно подключить сразу.

## 11. Реализация инкремента 1 — свой мир с целью (факты)

**Что построено:**
- `src/drone_simulator/worlds/follow_target.sdf` — наш мир (в git проекта). Копия штатного
  PX4 `default.sdf` (все системные плагины + физика + земля + солнце +
  `spherical_coordinates`) **плюс** включённая Fuel-модель человека
  `OpenRobotics/Standing person` (CC0) на позе `6 0 0 0 0 3.14159` (6 м впереди дрона по
  +X, лицом к нему), `static`. Меш скачивается с Fuel и кэшируется в `~/.gz/fuel` — в репо
  не коммитится, только URI в SDF.
- `scripts/run_px4_sitl.sh` — добавлен **standalone-режим** через `WORLD=<путь.sdf>`: скрипт
  сам поднимает `gz sim -r -s <world>` (+ GUI, если не `HEADLESS=1`), ждёт `clock`-топик,
  затем запускает `PX4_GZ_STANDALONE=1 make px4_sitl gz_x500_mono_cam` (PX4 цепляется к
  работающему Gazebo). `GZ_SIM_RESOURCE_PATH` дополняется моделями PX4 — чтобы сервер
  разрешил спавн `x500_mono_cam`. На выходе скрипт гасит поднятый им Gazebo (trap).
- `src/drone_simulator/setup.py` — ставит `worlds/*.sdf` в `share/drone_simulator/worlds/`.

**Почему standalone и почему `<world name="default">`** (важно):
- PX4 не отдаёт мир под подмену через env — при старте сорсит `gz_env.sh`, который перетирает
  `PX4_GZ_WORLDS` на свою папку. Поэтому штатный путь для своего мира — поднять Gazebo самим,
  а PX4 пустить с `PX4_GZ_STANDALONE=1`.
- В standalone PX4 **пропускает** блок определения имени работающего мира
  (`ROMFS/.../px4-rc.simulator`), и `PX4_GZ_WORLD` остаётся `default` (его жёстко задаёт
  make-цель `gz_x500_mono_cam`). `gz_bridge` цепляется к топикам `/world/default/*`. Значит
  имя `<world>` обязано быть `default`, иначе мост не найдёт сенсорику. Имя **файла** при
  этом описательное (`follow_target.sdf`).

**Как запускать (Терминал 1 заменяется на standalone-вариант):**
```bash
# Терминал 1 — Gazebo (наш мир) + PX4, одной командой:
WORLD=src/drone_simulator/worlds/follow_target.sdf scripts/run_px4_sitl.sh
#   варианты: HEADLESS=1 … (без GUI), GPU=nvidia … (гибридная графика, см. Фаза 2)
# Терминал 2 — агент + камера-мост (как в Фазе 2):
ros2 launch drone_simulator sim.launch.py
# Терминал 3 — что видит дрон:
ros2 run rqt_image_view rqt_image_view /camera/image
```
Остановка — `pxh> shutdown`, затем `scripts/stop_sim.sh` (в standalone PX4 **не владеет**
Gazebo, поэтому `stop_sim.sh` обязателен, чтобы погасить gz-сервер).

**Проверено (2026-06-26, headless smoke-тест только мира, без PX4):** `gz sim -s
follow_target.sdf` поднимает мир (`/world/default/clock`), Fuel-модель `Standing person`
скачивается и кэшируется (`~/.gz/fuel/.../standing person/3/meshes/standing.dae`), сущность
`target_person` присутствует (`gz model --list`), ошибок загрузки нет. `colcon build
--packages-select drone_simulator` — зелёный, мир ставится в share.

**Go/no-go инкр. 1 — ✅ пройден (2026-06-26, ручной прогон с GUI):** полный standalone-запуск
сработал — окно Gazebo показывает мир, человека и дрон (вид от третьего лица); PX4 подцепился
к нашему Gazebo, дрон заспавнился; в `rqt_image_view /camera/image` человек корректно виден
прямо перед дроном. Связка standalone-attach + камера подтверждена.

## 12. Реализация инкремента 2 — детектор (факты)

**Что построено:**
- `src/drone_perception/drone_perception/detector.py` — `ObjectDetector` (слим-порт логики
  из project_1 `src/detector.py`): YOLO над одним кадром, путь `.pt` (CPU/CUDA), без
  трекинга/ReID/onnx/engine. Вход `detect()` — кадр OpenCV BGR.
- `src/drone_perception/drone_perception/detector_node.py` — нода (заменила заглушку Фазы 1):
  `/camera/image` → `cv_bridge` (rgb8→**bgr8**) → `ObjectDetector` → фильтр по `target_classes`
  → выбор одной цели (`largest`/`closest`) → нормализованные `offset_x/offset_y/area_ratio`
  → `/perception/target` (`drone_interfaces/Target`, `header` = стамп кадра). Если
  `publish_annotated` — рисует bbox/крест/линию и публикует `/perception/image` (bgr8).
  Подписка на камеру — `qos_profile_sensor_data` (best-effort, под sensor-данные).
- `configs/perception/detector.yaml` — ros2-params (топики, `model_path`, `confidence`,
  `device`, `target_classes`, `selection`). `setup.py` ставит копию в share.
- `src/drone_bringup/launch/detector_demo.launch.py` — запуск только детектора с конфигом
  (для изолированной проверки поверх работающего sim).
- `package.xml` детектора: добавлены `sensor_msgs`, `cv_bridge`, `drone_interfaces`.
- Веса: `models/yolov8n.pt` (копия из project_1, gitignored через `*.pt`).

**Проверено мной (2026-06-26, без sim):**
- `colcon build` (interfaces+perception+bringup) зелёный; импорт `Target` и `DetectorNode` ок;
  YOLO грузится с `models/yolov8n.pt` на CUDA, инференс не падает.
- **Де-риск детекта:** прогон `yolov8n` по превью Fuel-модели `Standing person` →
  `person` с conf 0.81–0.92 на 4/5 ракурсов (см. §9).

**Как проверять (поверх работающего sim из §11):**
```bash
# Терминалы 1–2 — sim как в §11 (WORLD=… run_px4_sitl.sh  +  ros2 launch drone_simulator sim.launch.py)
# Терминал 3 — детектор (из корня проекта, три source):
ros2 launch drone_bringup detector_demo.launch.py
# Терминал 4 — проверка:
ros2 topic echo /perception/target            # detected: true, offset_x/area_ratio — живые
ros2 run rqt_image_view rqt_image_view /perception/image   # человек в красной рамке + зелёный крест центра
```

**Go/no-go инкр. 2 (на дисплее):** `/perception/target` отдаёт `detected=true` с осмысленным
`offset_x` (≈0, человек по центру; при сдвиге дрона/цели — меняется), и в `/perception/image`
виден bbox вокруг человека.

**⚠ Ловушка сборки (важно, решена): venv-shebang.** Первый запуск дал
`ModuleNotFoundError: No module named 'torch'`, хотя `(.venv)` активен. Причина: ноду
запускает сгенерированный entry-point, и интерпретатор задаёт его **shebang**, а не
активация venv в шелле. `ament_python` штампует в shebang тот python, под которым шёл
`colcon build`. Системный `colcon` — скрипт с shebang `/usr/bin/python3`, поэтому обычный
`colcon build` даёт ноды на системном python, который не видит `torch` (он только в `.venv`).
Решение — собирать colcon **под venv-python**; для этого заведён **`scripts/build.sh`**
(`.venv/bin/python $(command -v colcon) build "$@"`). Тогда shebang узла = `.venv/bin/python`,
который видит и `torch` (свой site-packages), и `rclpy` (через PYTHONPATH от `/opt/ros`, venv
создан с `--system-site-packages`). **Отныне собираем проект только `scripts/build.sh`**, иначе
ноды с pip-зависимостями снова сломаются. Проверено: `head -1` entry-point'а →
`#!.../.venv/bin/python`, нода грузит YOLO и доходит до «готов».
