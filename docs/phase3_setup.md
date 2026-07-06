# Фаза 3 — MVP «вижу → двигаюсь» (план реализации и решения)

> Назначение: подробно зафиксировать **что и как** реализуется в Фазе 3, принятые
> решения с обоснованием, порядок инкрементов (каждый — свой go/no-go), способы проверки
> и визуализации, ключевые ловушки. По мере реализации этот файл дополняется фактами
> (как `docs/phase2_setup.md`), а статусные строки помечаются ✅.
>
> Источник истины по архитектуре — `docs/project_plan.md` (§5, §8.9–8.10). Здесь — детали
> именно Фазы 3. Версии стека и sim-сторона — Фаза 2 (`docs/phase2_setup.md`), они не меняются.
>
> Статус: ✅ **Фаза 3 завершена** (2026-07-06). Инкременты 0 ✅ (`drone_interfaces`+`Target.msg`),
> 1 ✅ (свой мир + standalone, §11), 2 ✅ (`detector_node`: YOLO → `/perception/target`, §12),
> 3 ✅ (`follower_node`: offboard-цикл + P-регулятор, взлёт и реакция на фейк подтверждены, §13).
> Инкремент 4A — **✅ пройден** (2026-07-05, §14): `tracking_demo.launch.py` поднимает реальный
> пайплайн `follower_node`+`detector_node` с конфигами; над **статичной** целью дрон армится,
> взлетает, доворачивается к человеку и держит дистанцию (`offset_x→0`, `area_ratio≈area_target`).
> Инкремент 4B — **✅ пройден** (2026-07-06, §15): статичную цель заменил **ходячий** Gazebo
> `<actor>` (круг, смещённый вперёд); дрон детектит идущего человека и **физически летит следом**
> по сцене. Попутно вскрыты и учтены геометрический предел дистанции (горизонтальная камера) и
> стартовая гонка рендера (лечится `detector_delay_s`). Открытый блокер 4A закрыт: причина =
> контеншн рендера на гибридной графике (lockstep), лечение — **`GPU=nvidia`** для Gazebo (§9).
> **Петля «вижу → решаю → двигаюсь» замкнута. Следующее — Фаза 4** (см. `docs/project_plan.md`).

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
| **2** ✅ | `detector_node`: порт `ObjectDetector`, RGB→BGR, YOLO, выбор одной цели, публикация `/perception/target` (+ `/perception/image`). | человек детектится как `person`, bbox рисуется, `offset` реагирует на движение дрона ✅ (2026-06-26) |
| **3** ✅ | `follower_node`: offboard-цикл (стрим→offboard→arm с ретраем), P-регулятор, конфиг `configs/control/`. Тест с **фейковым** target. | дрон армится, offboard, взлёт, корректная реакция на фейк ✅ (2026-06-26) |
| **4A** ✅ | Полный пайплайн в `tracking_demo.launch.py` (детектор+контроллер+конфиги) над **статичной** целью + ручки диагностики arming (§14). Пройден 2026-07-05 с `GPU=nvidia` (§9, §14). | дрон взлетает, доворачивается к человеку, держит дистанцию ✅ |
| **4B** ✅ | Заменить статичную цель на **ходячего** `<actor>` (§15): круг, смещённый вперёд, + ретюн forward-канала под геометрию. Пройден 2026-07-06. | при движении цели — **летит следом** ✅ |

Порядок неслучаен: без объекта в мире (инкр. 1) детектор тестировать не на чем; контроллер
(инкр. 3) безопаснее обкатать на фейковом target до подключения живого детектора.

## 9. Ключевые ловушки и открытые детали Фазы 3

- **Детект меша человека** — ✅ де-рискнуто (2026-06-26): `yolov8n` детектит Fuel-модель
  `Standing person` как `person` с conf **0.81–0.92** на 4/5 ракурсов превью (промах лишь на
  нетипичном ракурсе). Камера смотрит вперёд на уровне роста — «хорошие» ракурсы. **Ходячий
  actor (`Mingfei/actor` `walk.dae`) перепроверен и тоже детектится** (§15, 2026-07-06):
  стоящий actor → `detected:true`, `offset_x≈0`; в движении удерживается на дуге/круге.
- **Как прокинуть свой мир в PX4** — ✅ решено (standalone, §11): сами поднимаем Gazebo с
  нашим SDF, PX4 цепляется (`PX4_GZ_STANDALONE`). `<world>` обязан называться `default`
  (в standalone PX4 не определяет имя работающего мира, `PX4_GZ_WORLD` остаётся `default`).
- **RGB/BGR** при `cv_bridge` (§6) — забыть конверсию = деградация YOLO.
- **Ориентация камеры** `x500_mono_cam` — ✅ подтверждено: монтаж `.12 .03 .242 0 0 0`,
  углы нулевые → камера смотрит **вперёд (+X body) горизонтально**, FOV≈100°. Цель ставим
  впереди по +X.
- **Offboard >2 Гц и порядок arm/offboard** (§5) — главные грабли управления PX4.
- **Кастомные msg → `ament_cmake`** (§3), не `ament_python` — это не ошибка структуры.
- **✅ ЗАКРЫТО (инкр. 4A, 2026-07-05): полный пайплайн не армился = контеншн рендера на
  гибридной графике.** Симптом: при одновременном запуске камера-моста + `detector_node` +
  `follower_node` (`tracking_demo`) PX4 SITL не армился (`Arming denied: Resolve system health
  failures first`, ранее — `Compass Sensor 0 missing` / `sensor_mag never published`), причём
  **одновременно деградировала камера** (`/perception/image` размытый, без горизонта/человека).
  **Причина** — ровно предсказанный сценарий lockstep: YOLO-инференс на встроенном GPU
  конкурировал с рендером Gazebo → шаг симуляции подтормаживал → в PX4 разом пропадала
  сенсорика, а камера отдавала «мусор». Оба симптома — один корень (GPU/рендер-контеншн).
  **Лечение — `GPU=nvidia`** для Gazebo (Терминал 1): рендер уходит на дискретную карту, и
  arming, и камера в норме — дрон армится, взлетает, доворачивается, держит дистанцию.
  Уточнение к прежней заметке: `GPU=nvidia` НЕ влиял на boot-строку `Ready for takeoff!`
  (она печатается и без него) — но оказался решающим именно для **arming** под нагрузкой.
  Ручки `detector_delay_s`/`detector_device` (§14) остаются для диагностики на других машинах.

## 10. Проверка и визуализация (как «пощупать» результат)

> 📋 Базовые команды проверки управления (pxh + ROS2, расшифровка кодов, типовые сценарии) —
> вынесены в шпаргалку **`docs/drone_commands.md`**.

### Подготовка терминала (обязательно для любых `ros2 …` команд)

Каждый **новый** терминал стартует с «голым» окружением — слои ROS2 не наследуются.
**Терминал 1** (`scripts/run_px4_sitl.sh`) подготовки НЕ требует (скрипт сам зовёт `make`).
Любой терминал, где запускаешь `ros2 launch` / `ros2 topic` / `ros2 run`, сперва подготовь
**из корня проекта**:
```bash
source /opt/ros/humble/setup.bash    # ros2 (иначе: ros2: command not found)
source install/setup.bash            # наши пакеты + px4_msgs (иначе: Package '…' not found / типы /fmu не разобрать)
source .venv/bin/activate            # python-зависимости (нужно detector_node: torch/ultralytics)
```
Маркер готовности: `echo $ROS_DISTRO` → `humble`; `ros2 pkg prefix drone_control` → путь в
`…/install/…`. Частая ошибка — забыть `source install/setup.bash` (даёт `Package … not found`
или пустой `/fmu/*`). Детали — `docs/phase2_setup.md`, раздел «Подготовка».

Sim поднимается так же: Терминал 1 — `scripts/run_px4_sitl.sh` (теперь с нашим миром),
Терминал 2 — `ros2 launch drone_simulator sim.launch.py` (агент + камера-мост), плюс пайплайн
Фазы 3 через `ros2 launch drone_bringup tracking_demo.launch.py`.

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
# Терминал 1 — Gazebo (наш мир) + PX4, одной командой (подготовка НЕ нужна):
WORLD=src/drone_simulator/worlds/follow_target.sdf scripts/run_px4_sitl.sh
#   варианты: HEADLESS=1 … (без GUI), GPU=nvidia … (гибридная графика, см. Фаза 2)
# Терминалы 2–3 (ROS2) — СПЕРВА подготовь окружение из корня проекта (см. §10):
#   source /opt/ros/humble/setup.bash && source install/setup.bash && source .venv/bin/activate
# Терминал 2 — агент + камера-мост:
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
# Терминалы 3–4 (ROS2) — СПЕРВА подготовь окружение из корня проекта (см. §10):
#   source /opt/ros/humble/setup.bash && source install/setup.bash && source .venv/bin/activate
# Терминал 3 — детектор:
ros2 launch drone_bringup detector_demo.launch.py
# Терминал 4 — проверка:
ros2 topic echo /perception/target            # detected: true, offset_x/area_ratio — живые
ros2 run rqt_image_view rqt_image_view /perception/image   # человек в красной рамке + зелёный крест центра
```

**Go/no-go инкр. 2 — ✅ пройден (2026-06-26):** человек детектится как `person`, bbox рисуется
в `/perception/image`, `offset_x` корректно реагирует на перемещение дрона (взлёт). Уверенность
bbox непостоянна — ожидаемо.

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

## 13. Реализация инкремента 3 — контроллер (факты)

**Что построено:**
- `src/drone_control/drone_control/follower_node.py` — offboard-контроллер (заменил заглушку
  Фазы 1). Таймер `loop_rate_hz` (20 Гц >2 Гц) каждый тик публикует `OffboardControlMode`
  (`velocity=True`) + `TrajectorySetpoint` — даже без цели (hover), иначе PX4 срывает offboard.
  Через `arm_after_s` однократно шлёт `VehicleCommand` DO_SET_MODE(offboard) + ARM (порядок §5).
- **P-регулятор (body-frame velocity):** высота — P по `target_altitude_m` (он же делает
  взлёт; NED z<0=вверх); `offset_x → yawspeed` (доворот, мёртвая зона); `area_ratio` vs
  `area_target → v_fwd` (вперёд/назад). body «вперёд» → NED по текущему `heading`
  (`TrajectorySetpoint.velocity` — в NED). Горизонталь включается только выше
  `min_follow_altitude_m`; при потере/устаревании цели (`target_timeout_s`) — hover.
  Знаки осей — параметры `yaw_sign`/`forward_sign` (frame-неоднозначность, §Ф3-5).
- `configs/control/follower.yaml` — все P-коэффициенты, лимиты, мёртвые зоны, знаки.
- `src/drone_bringup/launch/follower_demo.launch.py` — запуск только контроллера с конфигом.
- `package.xml` (+`px4_msgs`,`drone_interfaces`), `setup.py` (конфиг в share). QoS PX4 —
  best-effort + transient_local (как в офиц. примере px4_ros_com).

**Проверено мной (без PX4):** `scripts/build.sh` зелёный, shebang venv'овый; импорты
(`px4_msgs`, `FollowerNode`) ок; нода доходит до «готов» и через `arm_after_s` шлёт
offboard+arm. Имена топиков сверены с PX4 `dds_topics.yaml`: `/fmu/in/offboard_control_mode`,
`/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`, `/fmu/out/vehicle_local_position` —
совпадают.

**Как проверять (поверх sim + агента, БЕЗ детектора):**
```bash
# Терминал 1 — sim:   WORLD=src/drone_simulator/worlds/follow_target.sdf scripts/run_px4_sitl.sh
# Терминалы 2–4 (ROS2) — СПЕРВА подготовь окружение из корня проекта (см. §10):
#   source /opt/ros/humble/setup.bash && source install/setup.bash && source .venv/bin/activate
# Терминал 2 — агент: ros2 launch drone_simulator sim.launch.py
# Терминал 3 — контроллер:
ros2 launch drone_bringup follower_demo.launch.py
# Терминал 4 — ФЕЙКОВАЯ цель (правее центра, мелкий bbox → дрон должен довернуть вправо и вперёд):
ros2 topic pub -r 10 /perception/target drone_interfaces/msg/Target \
  '{detected: true, offset_x: 0.4, offset_y: 0.0, area_ratio: 0.05}'
```
**Правильная остановка** (контроллер каждую секунду заново держит OFFBOARD, поэтому
`commander land` при живом follower'е тут же перебивается — сперва гасим контроллер):
1. Ctrl-C в Терм.4 (фейк) → hover.
2. Ctrl-C в Терм.3 (follower) → setpoint'ы прекращаются, offboard больше не удерживается.
3. Терм.1: `pxh> commander land` (по желанию `commander disarm`).
4. `pxh> shutdown` → `scripts/stop_sim.sh`. В sim можно и без мягкой посадки: после шагов
   1–2 сразу `shutdown` + `stop_sim.sh`.

> ⚠️ Между прогонами ОБЯЗАТЕЛЬНО `scripts/stop_sim.sh` (standalone PX4 не владеет Gazebo —
> иначе остаются живые gz/агент, и следующий старт даёт `Arming denied: Resolve system health
> failures first` из-за конфликта двух симуляций). Проверка чистоты:
> `pgrep -af "gz sim|bin/px4|MicroXRCEAgent|parameter_bridge"` → пусто.

**Go/no-go инкр. 3 — ✅ пройден (2026-06-26, чистый старт):** дрон армится, входит в offboard,
взлетает на ~2.5 м и корректно реагирует на фейк (`offset_x=0.4` → доворот **вправо**,
`area_ratio=0.05` → ход **вперёд** = полёт по кругу); offboard не срывается; снятие фейка →
hover. Знаки осей оказались верны (`yaw_sign=forward_sign=1.0`). Первая попытка на «грязном»
старте дала `Arming denied` из-за висевших процессов прошлого прогона — лечится `stop_sim.sh`.

## 14. Реализация инкремента 4, шаг A — полный пайплайн в launch (факты)

**Что построено (2026-07-05):**
- `src/drone_bringup/launch/tracking_demo.launch.py` — **заменил заглушку Фазы 1**. Поднимает
  реальный пайплайн поверх работающего sim: `follower_node` (offboard P-цикл) +
  `detector_node` (YOLO → `/perception/target`), каждый со своим конфигом. Резолв конфига —
  как в demo-файлах: аргумент → `configs/<...>.yaml` (из корня проекта) → копия в share.
- **Порядок старта намеренный** (под открытый блокер §9): `follower_node` стартует в t=0 и
  успевает заармиться/взлететь до GPU-нагрузки; `detector_node` поднимается через
  `detector_delay_s` (`TimerAction`). Это и митигация, и инструмент диагностики lockstep-контеншна.
- **Диагностические ручки** (без правки файлов):
  - `detector_delay_s:=<сек>` — задержка старта детектора (`0.0` = одновременно, как в баге;
    напр. `10.0` = детектор после взлёта).
  - `detector_device:=cpu|cuda` — оверрайд `device` детектора (поверх конфига, поздний
    параметр перекрывает ранний) — проверка гипотезы «контеншн рендера на GPU».
  - `detector_config:=<путь>` / `follower_config:=<путь>` — явные пути к yaml.

**Проверено мной (без sim):** `scripts/build.sh --packages-select drone_bringup` зелёный;
`ros2 launch drone_bringup tracking_demo.launch.py --show-args` печатает все 4 аргумента;
логика резолва конфигов идентична уже проверенным `detector_demo`/`follower_demo`.

**Как проверять — рантайм-гейт 4A (статичная цель):**
```bash
# Терминал 1 — sim (наш мир). GPU=nvidia ОБЯЗАТЕЛЕН на гибридной графике (§9 — иначе не армится):
GPU=nvidia WORLD=src/drone_simulator/worlds/follow_target.sdf scripts/run_px4_sitl.sh
# Терминалы 2–4 (ROS2) — СПЕРВА подготовь окружение из корня (см. §10):
#   source /opt/ros/humble/setup.bash && source install/setup.bash && source .venv/bin/activate
# Терминал 2 — агент + камера-мост:
ros2 launch drone_simulator sim.launch.py
# Терминал 3 — ПОЛНЫЙ пайплайн (детектор+контроллер):
ros2 launch drone_bringup tracking_demo.launch.py
# Терминал 4 — наблюдение:
ros2 topic echo /perception/target     # detected: true, offset_x/area_ratio — живые
ros2 run rqt_image_view rqt_image_view /perception/image    # человек в рамке + крест центра
```
Ожидаемо (гейт 4A): дрон армится, взлетает на `target_altitude_m`, **доворачивается к
человеку** (`offset_x → 0` в `rqt_plot`/`echo`) и **держит дистанцию** (`area_ratio →
area_target=0.15`). В окне Gazebo — главное подтверждение.

**Если не армится (открытый блокер §9) — диагностика по гипотезам, по одной:**
1. Чистота перед стартом — обязательно (см. ⚠ ниже): `scripts/stop_sim.sh`,
   `pgrep -af "gz sim|bin/px4|MicroXRCEAgent|parameter_bridge"` → пусто.
2. **Тайминг старта** (детектор после взлёта): `tracking_demo.launch.py detector_delay_s:=10.0`.
   Армится → причина в одновременной нагрузке на старте (lockstep/контеншн), а не в самом детекторе.
3. **GPU-контеншн**: `tracking_demo.launch.py detector_device:=cpu` (детектор на CPU).
   Армится → виновата нагрузка рендера/инференса на гибридной графике.
4. **GPU для Gazebo**: Терминал 1 с `GPU=nvidia …`.
   Сверять с `commander check` (pxh) и `ros2 topic hz /fmu/out/sensor_combined` — пропадает ли
   сенсорика. Цель — **локализовать**, какая ручка снимает `Compass Sensor 0 missing`, а не
   включить всё разом.

**Правильная остановка** (как в инкр. 3): Ctrl-C в Терм.3 (пайплайн) → setpoint'ы
прекращаются; Терм.1 `pxh> commander land` (опц.) → `shutdown` → `scripts/stop_sim.sh`.
⚠️ Между прогонами `scripts/stop_sim.sh` ОБЯЗАТЕЛЕН (standalone PX4 не владеет Gazebo).

**Go/no-go инкр. 4A — ✅ пройден (2026-07-05):** с `GPU=nvidia` дрон армится, взлетает,
доворачивается к человеку и держит дистанцию. `ros2 topic echo /perception/target` →
`detected: true`, `offset_x ≈ −0.02` (цель по центру), `area_ratio ≈ 0.155` при
`area_target=0.15` (дистанция удержана); в `/perception/image` человек по центру, крупным
планом. Без `GPU=nvidia` — не армился + деградировала камера (диагноз и причина: §9).

**Шаг B (после прохождения 4A):** в `follow_target.sdf` заменить статичный `Standing person`
на Gazebo `<actor>` с анимацией ходьбы и траекторией (waypoints). Перепроверить детект меша
actor'а YOLO (§9 — другой меш/анимация). Гейт 4B: дрон следует за идущей целью.

## 15. Реализация инкремента 4, шаг B — ходячая цель (факты)

**Что построено (2026-07-06):** статичный `Standing person` в `follow_target.sdf` заменён на
Gazebo `<actor name="target_person">` — скелетная анимация ходьбы + `<script><trajectory>` из
waypoints (штатный механизм Gazebo, **без плагинов и без правки кода нод**; сервер интерполирует
позу актёра между точками). Меш `walk.dae` из Fuel-модели `Mingfei/models/actor` (тот же, что в
штатном `gz-sim8/worlds/actor.sdf`) — скачивается при первом запуске, кэшируется в `~/.gz/fuel`,
в git не коммитится. `z=1.0` в позах — начало скелета на уровне бёдер, ступни на земле. У `<actor>`
нет `<static>`/физики/коллизий — чисто визуальная цель, что для камеры/детектора и нужно.

**Порядок отладки (два под-шага, как договорились):**
- **4B-1 (де-риск, стоящий actor):** одна поза (две одинаковые waypoint'ы) в 6 м впереди, лицом
  к дрону. ✅ Подтвердило оба риска §9: (а) камера-сенсор actor'а **видит**, (б) YOLO ловит
  `walk.dae` как `person` (`detected:true`, `offset_x≈0`, `offset_y≈−0.12`, `area_ratio≈0.006`).
- **4B-2 (ходьба):** actor идёт по **замкнутому кругу, СМЕЩЁННОМУ ВПЕРЁД** (центр `(8,0)`, R=3,
  ~0.25 м/с, ~72 с на круг, yaw = касательная, непрерывно растущий → плавное вращение, `loop`
  без телепорта). Дрон детектит и **физически летит следом** по сцене. ✅ Гейт пройден.

**⚠ Ловушка Ф3-6 — «дрон крутится на месте» (важно, решена).** Первый вариант траектории — дуга,
центрированная на **точке старта дрона `(0,0)`**. Тогда цель **вращается вокруг дрона**, и тому
для удержания хватает одного **yaw** — дрон крутится, никуда не летит. **Правило: путь цели
должен быть смещён от точки старта дрона** (цель перемещается сквозь сцену, а не орбитой вокруг
дрона) — только тогда включается forward и дрон реально следует. Финальный круг центрирован в
`(8,0)`, не в `(0,0)`.

**⚠ Ловушка Ф3-7 — геометрический предел дистанции (горизонтальная камера + наземная цель).**
Камера `x500_mono_cam` смотрит **горизонтально** (§9), дрон висит на `target_altitude=2.5 м`,
человек на земле. Ближе ~3 м угол склонения `atan(2.5/d) > VFOV/2 (~41°)` → цель **уходит из
нижнего края кадра** (`offset_y → +0.22…`), `area_ratio` упирается в **~0.028** и выше не растёт.
Поэтому `area_target=0.15` (значение 4A для крупного статичного меша) для ходячего actor'а
**недостижимо** — форвард насыщался, дистанция не регулировалась. **Учтено ретюном** (`configs/
control/follower.yaml`): `area_target 0.15→0.02` (держаться чуть снаружи «стены»), `area_deadband
0.02→0.004` (мёртвая зона перестала «съедать» весь сигнал при мелких `area`), `kp_forward 3→18`
(масштаб отклика под мелкие величины `area`, т.к. `area~1/d²` нелинейна). Полноценная развязка
нелинейности (линейный прокси дистанции `~1/√area`, подстройка высоты по `offset_y`) — задача
Фазы 4.

**⚠ Ловушка Ф3-8 — стартовая гонка рендера (уточнение к §9/§4A).** На **чистом** старте
(`stop_sim.sh` выполнен, процессов нет) полный пайплайн иногда всё равно поднимался с
**размытой** `/perception/image` и пустым `/perception/target` при **здоровом** PX4
(`Ready for takeoff!` есть). Причина — не грязный старт, а **timing-гонка §9**: `detector_node`
по умолчанию стартует одновременно с камерой (`detector_delay_s=0`), и инициализация + первый
инференс YOLO на NVIDIA конкурируют с прогревом рендера камеры-сенсора; иногда камера «голодает»
и отдаёт мусор. **Лечение — `detector_delay_s:=8`** (детектор поднимается после прогрева камеры и
взлёта; §14 — эта ручка ровно для того). Диагностический маркер — **чёткость `/perception/image`
проверять до слежения**; размытие = этот контеншн, а не баг пайплайна.

**Как проверять — рантайм-гейт 4B:**
```bash
# чистота: scripts/stop_sim.sh ; pgrep -af "gz sim|bin/px4|MicroXRCEAgent|parameter_bridge" → пусто
# Терминал 1 — sim (наш мир, GPU=nvidia ОБЯЗАТЕЛЕН на гибридной графике, §9):
GPU=nvidia WORLD=src/drone_simulator/worlds/follow_target.sdf scripts/run_px4_sitl.sh
# Терминалы 2–4 — окружение из корня (см. §10): source /opt/ros/humble/setup.bash && source install/setup.bash && source .venv/bin/activate
# Терминал 2 — агент + камера-мост:
ros2 launch drone_simulator sim.launch.py
# Терминал 3 — пайплайн С ЗАДЕРЖКОЙ детектора (лечит гонку Ф3-8):
ros2 launch drone_bringup tracking_demo.launch.py detector_delay_s:=8
# Терминал 4 — СНАЧАЛА чёткость кадра, потом слежение:
ros2 run rqt_image_view rqt_image_view /perception/image     # кадр ЧЁТКИЙ, человек в рамке
ros2 topic echo /perception/target                           # detected:true, offset_x/y живут
```

**Что менялось (файлы):** только `src/drone_simulator/worlds/follow_target.sdf` (actor+траектория)
и `configs/control/follower.yaml` (ретюн `area_target`/`area_deadband`/`kp_forward`). Код нод —
**без изменений** (движущаяся цель обрабатывается тем же P-циклом). Пересборка не требуется:
`WORLD=` читает SDF из исходника, launch читает `configs/` из корня.

**Go/no-go инкр. 4B — ✅ пройден (2026-07-06):** с `GPU=nvidia` и `detector_delay_s:=8` дрон
армится, взлетает, ловит идущего по кругу человека и **летит за ним по сцене** (видимое
перемещение, не спин); `detected:true` устойчиво, `offset_x/offset_y` колеблются у нуля,
`area_ratio` живёт около `area_target`. В `rqt_image_view /perception/image` дрон удерживает
человека в центре кадра. **Гейт Фазы 3 достигнут — перцепция физически управляет аппаратом.**
