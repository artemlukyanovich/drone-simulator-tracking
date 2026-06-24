# drone-simulator-tracking — План, решения и принципы

> Назначение файла: единая точка входа в проект для **человека-разработчика** и
> **ИИ-агента**. Здесь зафиксированы принятые решения (с обоснованием), архитектура,
> принципы реализации, план развития и ключевые технические ловушки.
> Если что-то в коде противоречит этому документу — сначала сверяемся здесь.

Дата создания: 2026-06-09. Язык проекта (документация): русский. Язык кода/идентификаторов: английский.

---

## 1. Назначение проекта

Научиться **управлять системой (дроном), а не просто распознавать картинку**, и
приблизиться к работе с реальными UAV. Проект — это связка «вижу → решаю → двигаюсь»:

```
Дрон → Камера → Детектор → Трекинг → Решение → Движение
```

Главная идея экосистемы:
- **project_1** (`real-time-object-counter`) = *видеть объект* (CV/perception-лаборатория).
- **project_2** (этот проект) = *управлять дроном по результатам зрения*.

## 2. Контекст и место в экосистеме

```
ai_drones/                         ← локальный workspace (НЕ git-репозиторий)
├── project_1/assets/                     ← данные, видео, картинки проекта 1 — локально, НЕ в git
├── project_2/assets/                     ← данные, видео, картинки проекта 2 — локально, НЕ в git
├── project_1/real-time-object-counter/   ← отдельный git-репо (CV-лаборатория)
└── project_2/drone-simulator-tracking/   ← ЭТОТ проект, отдельный git-репо
```

`ai_drones/` — это просто папка-контейнер на диске, а не репозиторий. Каждый подпроект —
самостоятельный git-репозиторий со своим remote, своим окружением (venv) и своим темпом
развития.

## 3. Принятые решения (с обоснованием)

| # | Решение | Почему так, а не иначе |
|---|---------|------------------------|
| Р1 | **ROS2-first** (не MAVSDK-first) | Это индустриальный стандарт и «правильный» способ строить подобные системы. Даёт знания, переносимые на реальные UAV. Бонус: камера приходит как стандартный ROS-топик `sensor_msgs/Image`, что снимает самый сложный кусок MVP (вытаскивание кадра из симулятора). |
| Р2 | **Multi-repo**, не монорепо | project_1 уже отдельный git-репо со своим remote. Слить его в монорепо = git-хирургия (subtree/удаление `.git`). К тому же project_2 — это colcon-workspace, структурно другой зверь, чем plain-python репо project_1. Два репо разной природы живут чище. |
| Р3 | **Перцепцию из project_1 переносим как логику, оборачивая в ROS2-ноды.** Не `pip install`, не общий пакет. | Дрону нужна перцепция под симуляторную картинку, реалтайм-бюджет и свои классы — она *должна* разойтись с лабораторией-счётчиком. Дивергенция здесь здоровая, а не вредная. Редкие настоящие багфиксы переносим руками. |
| Р4 | **Отдельное окружение (venv) на проект** | Зависимости sim/ROS2 (`rclpy`, `px4_msgs`, ros_gz) конфликтуют по версиям с CV-стеком project_1. |
| Р5 | **assets/ — локально, не в git** | Тяжёлые видео/датасеты не место в репозитории. |
| Р6 | **Связь с PX4 — нативно через uXRCE-DDS + `px4_msgs`** (ROS2-путь). MAVSDK — опционально/позже. | В ROS2-first родной способ — публиковать/читать uORB-топики PX4 напрямую через DDS-мост. MAVSDK становится не нужен для базового цикла. |
| Р7 | **Окружение — нативное: системный ROS2 Humble (apt) + venv с `--system-site-packages`.** Не Docker (пока). | Весь стек (PX4/Gazebo/ROS2/DDS) нативно работает на Jammy-базе; Docker не нужен для интеграции, а его GUI(X11)+GPU-проброс на ноутбуке — лишняя возня. Единственный папкорез — Mint вместо Ubuntu (форсить codename `jammy`). Docker — позже, под удалённое развёртывание/CI. venv с `--system-site-packages`, чтобы ноды видели и системный `rclpy`, и pip-зависимости (torch/ultralytics). |

## 4. Технологический стек

- **Gazebo** (новый, бывший Ignition / `gz-sim`; не Gazebo Classic) — физика и сенсоры.
- **PX4 SITL** — настоящий автопилот в software-in-the-loop.
- **ROS2** — middleware, ноды, топики. Дистрибутив — LTS.
- **uXRCE-DDS** (`micro-xrce-dds-agent`) + **`px4_msgs`** / `px4_ros_com` — мост PX4 ↔ ROS2.
- **`ros_gz_bridge`** — проброс камеры Gazebo в ROS2 как `sensor_msgs/Image`.
- **YOLO (ultralytics)** — детектор (логика переносится из project_1).
- **MAVSDK** — опционально, позже.

### Зафиксированная тройка версий (Фаза 0, риск закрыт)

Машина разработки: **Linux Mint 21.3** (база Ubuntu 22.04 «Jammy»), Python 3.10.12,
NVIDIA RTX 3050 Ti (драйвер 580 → CUDA 12.x доступна).

| Компонент | Версия | Примечание |
|---|---|---|
| База ОС | Ubuntu 22.04 (Mint 21.3) | при добавлении apt-репо форсить codename `jammy` (Mint отдаёт `virginia`) |
| ROS2 | **Humble Hawksbill** (LTS) | штатен под 22.04 / Python 3.10 |
| Gazebo | **Harmonic** (gz-sim8, LTS) | мост к Humble — пакет `ros-humble-ros-gzharmonic` |
| PX4 | **v1.15.x** | поддерживает Humble + uXRCE-DDS + Harmonic |
| Мост PX4↔ROS2 | Micro-XRCE-DDS-Agent + `px4_msgs` | ветку `px4_msgs` брать под выбранный релиз PX4 |
| Python | 3.10 | системный, общий с ROS2 |

> Точное соответствие PX4 ↔ Gazebo (Harmonic vs Garden) подтвердить по docs.px4.io при
> установке (Garden EOL, целимся в Harmonic). Дубль этой таблицы — в `configs/simulator/stack.md`.

## 5. Архитектура

### Слои (поток данных)

```
Gazebo (физика + камера)
   │  image (gz topic)
   ▼
ros_gz_bridge ──► /camera/image  (sensor_msgs/Image)
   ▼
drone_perception: detector_node ──► /perception/target  (bbox + смещение от центра)
   ▼
drone_control: follower_node ──► OffboardControlMode + TrajectorySetpoint
   │  (через uXRCE-DDS / px4_msgs)
   ▼
PX4 SITL ──► моторы в Gazebo
```

Ключевая ментальная модель ROS2: ноды **не вызывают друг друга функциями**, а обмениваются
сообщениями через топики. Pipeline собирается не кодом, а **launch-файлом** в `drone_bringup`.

### Карта нод и топиков (целевая)

| Нода | Пакет | Подписан на | Публикует |
|------|-------|-------------|-----------|
| (мост) `ros_gz_bridge` | — | gz camera | `/camera/image` |
| `detector_node` | drone_perception | `/camera/image` | `/perception/target` |
| `follower_node` | drone_control | `/perception/target`, телеметрия PX4 | setpoints в PX4 |

## 6. Структура репозитория (colcon workspace)

```
drone-simulator-tracking/          ← это WORKSPACE, а не обычный проект
├── src/                           ← сюда складываются ROS2-ПАКЕТЫ
│   ├── drone_simulator/           ← запуск SITL, мосты, описание мира/дрона
│   ├── drone_perception/          ← detector_node и т.п. (логика из project_1)
│   ├── drone_control/             ← follower_node, PID, offboard-цикл
│   └── drone_bringup/             ← launch-файлы, оркестрация всего пайплайна
│
├── configs/                       ← НЕ ROS-специфично, обычные конфиги
│   ├── simulator/                 ← версии стека, параметры мира, IP/портов DDS
│   ├── perception/                ← пороги детектора, веса, классы
│   └── control/                   ← коэффициенты P/PID, лимиты скоростей
│
├── outputs/                       ← результаты прогонов (в gitignore)
│   ├── videos/  logs/  telemetry/
│
├── scripts/                       ← run_px4_sitl.sh, run_bridge.sh, demo и т.п.
├── docs/                          ← этот файл и прочая документация
│
├── build/  install/  log/         ← генерирует colcon, НЕ коммитить (в gitignore)
├── README.md  CLAUDE.md  .gitignore
```

Запуск (общая схема): `colcon build` → `source install/setup.bash` → `ros2 launch drone_bringup ...`.

> ✅ Фаза 1 (2026-06-10): все четыре каталога под `src/` оформлены как настоящие
> **ament_python-пакеты** (`package.xml` format 3 + `setup.py` + `setup.cfg` +
> `resource/<pkg>` + вложенный python-пакет). `drone_perception`/`drone_control`
> несут ноды-заглушки (heartbeat-логгеры), `drone_simulator` — каркас без нод (Фаза 2),
> `drone_bringup` — `launch/tracking_demo.launch.py`. `colcon build` зелёный.

## 7. Принципы реализации (для человека и ИИ-агента)

**ROS2-конвенции**
- Имена **пакетов** и **нод** — `snake_case` без дефисов (дефис допустим только в имени
  репозитория/workspace). Внутри ament_python код лежит в `src/<пакет>/<пакет>/`.
- Каждая нода — отдельный процесс с одной зоной ответственности. Связь — только через
  топики/сервисы, не через прямые импорты между нодами.
- Сборка пайплайна — через **launch-файлы** в `drone_bringup`, а не хардкодом.

**Управление PX4 (критично)**
- Offboard-режим PX4 **отваливается, если setpoint'ы не идут непрерывно с частотой >2 Гц**.
  Контроллер — это **постоянный цикл**, который каждый тик публикует setpoint (даже «висеть» =
  нулевая скорость), а не разовая команда «сместиться влево».
- Перед входом в offboard PX4 требует, чтобы поток setpoint'ов уже шёл. Соблюдать порядок:
  стримим setpoint'ы → переключаем режим → arm.
- Явно фиксировать систему координат (body vs NED) при переводе смещения bbox в setpoint.

**Гигиена репозитория**
- Никогда не коммитить `build/`, `install/`, `log/`, содержимое `outputs/`, веса моделей,
  виртуальные окружения, `.idea/`.
- Конфигурация — в `configs/`, **отдельно от кода**. Магических чисел в коде не держим.

**Рабочий процесс с репозиторием**
- **Рабочий корень — сам проект** (`…/project_2/drone-simulator-tracking/`), его и открываем
  в PyCharm и в Claude Code. Это одновременно корень git, корень venv и корень colcon-workspace.
  `ai_drones/` сверху НЕ открываем как проект — это лишь файловый контейнер (см. Р2/Р4).
- `CLAUDE.md` подхватывается Claude Code из корня проекта автоматически; он — краткая
  операционная выжимка, а **источник истины — этот файл** (`docs/project_plan.md`).
- Если нужно подсмотреть код project_1 — открываем его **отдельной** сессией/окном, не оборачивая
  оба проекта в один корень.

**IDE: чтобы PyCharm видел `rclpy` / ROS2 (важная ловушка)**
- `rclpy` и прочие ROS2-пакеты лежат в `/opt/ros/humble/...` и подключаются **через
  `PYTHONPATH`** (его ставит `source /opt/ros/humble/setup.bash`), а **не** через site-packages.
  Поэтому флаг venv `--system-site-packages` их НЕ открывает, и PyCharm, запущенный «голым»
  (из меню/иконки), показывает `No module named 'rclpy'`. Это ожидаемо.
- **Принятое решение (2026-06-10): Вариант 2 — вручную добавить ROS2-пути в интерпретатор.**
  `Settings → Project → Python Interpreter → Show All → выбрать .venv → Interpreter Paths
  (значок дерева) → +` и добавить:
  `/opt/ros/humble/local/lib/python3.10/dist-packages` и
  `/opt/ros/humble/lib/python3.10/site-packages`. Это снимает подсветку и даёт автодополнение.
- ⚠️ **Это только индексатор IDE (подсветка/автодополнение), не окружение выполнения.** Запуск
  кода (терминал, `ros2 run`/`ros2 launch`, Run-конфиги) **по-прежнему требует** `source
  /opt/ros/humble/setup.bash` → `source install/setup.bash`, активный `.venv` — необходимость
  та же, что и без правки путей (рантайму нужны `LD_LIBRARY_PATH`/`AMENT_PREFIX_PATH`,
  а не пути индексатора). Порядок между этими source-командами не важен — нужны лишь оба
  слоя; детали и проверка — в `configs/simulator/stack.md`. Минус подхода: при добавлении новых ROS-пакетов (напр. `px4_msgs`) их
  пути в интерпретатор придётся дописывать руками.
- Альтернатива (не выбрана): запускать IDE через `scripts/launch_pycharm.sh`, который сорсит
  окружение в сам процесс PyCharm — тогда и Run-конфиги, и встроенный терминал работают без
  ручного source, а индексатор видит всё разом. Оставлено на потом.

**Общее**
- Документация — на русском, идентификаторы/код — на английском.
- Любое значимое архитектурное решение — фиксируем в §3 этого файла.
- Корневые файлы репозитория: `README.md` (обзор, англ.), `CLAUDE.md` (операционная выжимка для
  ИИ-агента, англ.), `.gitignore`. Подробный план и решения — здесь, в `docs/project_plan.md` (рус.).

## 8. План развития

**Фаза 0 — окружение и версии**
1. Зафиксировать совместимую тройку PX4 + Gazebo + ROS2 LTS, записать в `configs/simulator/`.
2. Поднять отдельный venv проекта поверх системного python3 (3.10) с `--system-site-packages` —
   скриптом `scripts/setup_venv.sh` (идемпотентен, ставит зависимости из `requirements.txt`).
   `git init` + новый remote.
3. Базовый `.gitignore`, `README.md`, `CLAUDE.md`.
4. Установить системный стек (ROS2 Humble, Gazebo Harmonic, ros_gz) через apt + собрать
   Micro-XRCE-DDS-Agent из исходников (apt-пакета нет, snap на Mint заблокирован) —
   скриптом `scripts/install_system_deps.sh` (форсит codename `jammy`). Это база для Фазы 2;
   pip-зависимости (`torch`, `ultralytics`) идут в venv через `requirements.txt`.

**Фаза 1 — каркас ROS2** ✅ (2026-06-10)
4. ✅ Оформить `src/*` как настоящие ament-пакеты (`package.xml`, `setup.py`).
5. ✅ Пустые ноды-заглушки (heartbeat-логгеры) + первый launch-файл
   `tracking_demo.launch.py` в `drone_bringup`. `colcon build` зелёный; `ros2 launch`
   поднимает `/detector_node` и `/follower_node`. Кастомный интерфейс
   `/perception/target` отложен до Фазы 3 (пока нодам нечего публиковать).

**Фаза 2 — де-риск симулятора (раньше пайплайна!)** ✅ (2026-06-11)
6. ✅ PX4 SITL + Gazebo (`scripts/run_px4_sitl.sh`, модель `gz_x500_mono_cam`): SITL дошёл до
   `Ready for takeoff!`, `commander takeoff` → взлёт (`vehicle_local_position.z ≈ −1.96`).
7. ✅ uXRCE-DDS мост (`scripts/run_xrce_agent.sh` + `px4_msgs` `release/1.15` в `src/`):
   **43** `/fmu/*` топика в `ros2 topic list`, телеметрия `vehicle_local_position` на 100 Гц.
8. ✅ Камера через `ros_gz_bridge` (`drone_simulator/launch/sim.launch.py` +
   `configs/simulator/camera_bridge.yaml`): `/camera/image` на ~30 Гц, 640×480 rgb8. ← go/no-go **пройден**.

> Зафиксированный стек Фазы 2: PX4 **v1.15.4** (`99c40407ff`, в `~/src/`, вне git),
> px4_msgs `release/1.15` (`a1045ec`), Micro-XRCE-DDS-Agent v2.4.3, Gazebo Harmonic 8.13.
> Пошаговое воспроизведение без ИИ — **`docs/phase2_setup.md`**. Детали стека — `configs/simulator/stack.md`.

**Фаза 3 — MVP «вижу → двигаюсь»** (детальный план — `docs/phase3_setup.md`)
9. `detector_node`: подписка на `/camera/image`, YOLO, публикация bbox + смещения от центра кадра.
10. `follower_node`: непрерывный offboard-цикл, P-регулятор:
    - target левее/правее центра → yaw/смещение в сторону;
    - bbox маленький → вперёд (приблизиться); большой → назад (отдалиться).

> Решения Фазы 3 (детали и обоснование — `docs/phase3_setup.md` §2): цель — **движущийся
> человек** (Gazebo `actor`, класс COCO `person`); интерфейс — **свой
> `drone_interfaces/Target.msg`** (нормализованные offset+размер), не `vision_msgs`; степени
> свободы MVP — **yaw + вперёд/назад, высота фикс.**; координаты setpoint'ов — **body-frame
> velocity**. Детектор переносится из project_1 как логика (без трекинга/ReID — Фаза 4).

**Фаза 4 — углубление**
11. P → PID (`configs/control/`), логирование телеметрии в `outputs/telemetry/`.
12. Перенос/доработка трекинга и (опц.) ReID из project_1 под дрон.
13. Запись демо-видео, бенчмарки.

## 9. Ключевые технические ловушки (gotchas)

- **Версии стека** жёстко связаны (PX4/Gazebo/ROS2) — закрыть в первую очередь (§4, §11).
- **Offboard >2 Гц** — иначе режим срывается (§7).
- **Порядок arm/offboard** — стрим setpoint'ов до переключения режима.
- **Система координат** bbox→setpoint (body vs NED) — фиксировать явно.
- **DDS-мост** (uXRCE-DDS agent) должен быть запущен, иначе PX4-топики не появятся в ROS2.
- **Двойная вложенность** `src/<пакет>/<пакет>/` — требование ament_python, не ошибка.

## 10. Глоссарий

- **Workspace** — корень colcon-проекта; содержит `src/`, после сборки появляются `build/install/log`.
- **Package (ament-пакет)** — единица ROS2: `package.xml` + `setup.py` + python-код.
- **Node** — исполняемый процесс ROS2; общается через топики.
- **Topic** — именованный канал сообщений (pub/sub).
- **SITL** — Software-In-The-Loop: автопилот PX4 работает как программа, без железа.
- **Offboard** — режим PX4, когда управление идёт от внешних setpoint'ов (наш контроллер).
- **uXRCE-DDS** — мост, через который PX4 публикует/принимает топики в DDS/ROS2.

## 11. Открытые вопросы / TODO

- [x] Зафиксировать версии: ROS2 **Humble**, Gazebo **Harmonic**, PX4 **v1.15.x** (см. §4).
- [x] Выбрать окружение: **нативно — системный ROS2 + venv `--system-site-packages`** (Р7).
- [x] Подтвердить точное соответствие PX4 ↔ Gazebo: **PX4 v1.15.4 собирается и летает на Gazebo
      Harmonic 8.13** (Фаза 2, 2026-06-11); мост — `ros-humble-ros-gzharmonic`.
- [x] Определить, какие именно модули из project_1 переносим первыми: **детектор**
      (`ObjectDetector` из `src/detector.py`); трекинг/ReID — Фаза 4 (Фаза 3, см. `phase3_setup.md`).
- [ ] Решить: системный ROS2 + venv, или Docker — **позже**, при удалённом развёртывании/CI.
- [x] Решить набор классов/целей: **класс `person`** — дрон следует за движущимся человеком
      (Gazebo `actor`) (Фаза 3, см. `phase3_setup.md` §2).
