# Фаза 2 — воспроизведение установки симулятора (без ИИ-агента)

> Назначение: пошагово повторить всё, что сделано в Фазе 2, руками. Каждый инкремент —
> самостоятельный go/no-go. Версии — источник истины `configs/simulator/stack.md` и
> `docs/project_plan.md` §4. Команды даны для машины разработки (Mint 21.3 / Jammy).
>
> Предполагается, что **Фаза 0 уже выполнена**: системный слой стоит
> (`scripts/install_system_deps.sh` — ROS2 Humble, Gazebo Harmonic, ros_gz,
> MicroXRCEAgent), venv создан (`scripts/setup_venv.sh`). Проверка: `gz sim --version`
> → 8.x, `command -v MicroXRCEAgent` непустой, `echo $ROS_DISTRO` после
> `source /opt/ros/humble/setup.bash` → `humble`.

---

## Внешние зависимости (вне git) и что будет при их удалении

Фаза 2 опирается на тяжёлый сторонний код, который **не коммитится** в репозиторий и
живёт двумя группами (важно не путать две разные папки `src/`):

- `~/src/` — личная «мастерская» собранных-из-исходников инструментов, **полностью вне**
  проекта (не colcon-workspace): `PX4-Autopilot`, `Micro-XRCE-DDS-Agent`.
- `<проект>/src/px4_msgs` — внешний код, но **физически внутри** colcon-workspace, потому
  что colcon обязан его собрать (генерирует ROS2-типы PX4). В `.gitignore`, в наш репо не идёт.

Это сторонние тулзы/SDK, а не наш код: у каждого свой git и большой объём, поэтому держим
снаружи и **восстанавливаем скриптами/командами ниже**. Ничего из нашей работы при их
удалении не теряется — только пере-скачать/пересобрать.

| Артефакт | Расположение | Нужен для | Удалишь → | Восстановление |
|---|---|---|---|---|
| `PX4-Autopilot` | `~/src/PX4-Autopilot` | сам симулятор+автопилот (SITL) | `run_px4_sitl.sh` не запустится, симуляции нет | clone + build — §A.1–A.3 |
| `Micro-XRCE-DDS-Agent` (исходники) | `~/src/Micro-XRCE-DDS-Agent` | пересборка агента | **ничего** — бинарь уже в системе | нужен лишь для пересборки |
| `MicroXRCEAgent` (бинарь) | `/usr/local/bin` | мост PX4↔ROS2 | нет `/fmu/*` топиков в ROS2 | пересобрать — `scripts/install_system_deps.sh` |
| `px4_msgs` | `src/px4_msgs` (gitignored) | сборка типов PX4, `ros2 echo /fmu/*` | `colcon build` не соберёт типы | clone + build — §B.1–B.2 |

> Связь с PX4-Autopilot — **рантаймовая, не через код**: проект не импортирует и не
> собирает его внутрь workspace, а запускает как отдельный процесс (`scripts/run_px4_sitl.sh`
> делает `cd ~/src/PX4-Autopilot && make px4_sitl …`), который общается с нашей ROS2-стороной
> по сети (uXRCE-DDS, UDP 8888) и с Gazebo.

---

## Инкремент A — PX4 SITL + Gazebo (план §8.6) ✅

Цель: автопилот PX4 в SITL поднимает дрон `x500_mono_cam` в Gazebo Harmonic и взлетает.

### A.1. Склонировать PX4-Autopilot (вне репозитория проекта)

PX4 живёт в `~/src/` рядом с Micro-XRCE-DDS-Agent — **не** внутри git проекта (его не
коммитим). Берём зафиксированный тег `v1.15.4`:

```bash
mkdir -p ~/src
git clone -b v1.15.4 --recurse-submodules --jobs 4 \
  https://github.com/PX4/PX4-Autopilot.git ~/src/PX4-Autopilot
```

Проверка: `git -C ~/src/PX4-Autopilot describe --tags` → `v1.15.4`.
(Зафиксированный commit: `99c40407ff`.)

### A.2. Поставить build-зависимости PX4 в venv

Сборка PX4 использует python-инструменты (`empy`, `kconfiglib`, `jinja2`, …). Ставим их
из родного `Tools/setup/requirements.txt` PX4 **в наш venv** (правильные пины, в т.ч.
`empy==3.3.4`):

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate                 # из корня проекта drone-simulator-tracking
pip install -r <(sed 's/\.\*//g' ~/src/PX4-Autopilot/Tools/setup/requirements.txt)
pip install pyros-genmsg                   # требуется генератору сообщений, нет в requirements
```

Две ловушки (обе — не ошибки):
- `sed 's/\.\*//g'` чинит несовместимый с новым pip спек `matplotlib>=3.0.*` (суффикс `.*`
  допустим только с `==`/`!=`). Санитайзим на лету, не правя файл PX4.
- Пакет `empy` **импортируется как модуль `em`**, а не `empy`. Проверка успеха:
  `python3 -c "import em, kconfiglib, jinja2, toml; print('ok')"`.

### A.3. Собрать прошивку SITL

Первая сборка длинная (минуты). Цель `px4_sitl_default` собирает прошивку без запуска GUI:

```bash
cd ~/src/PX4-Autopilot
make px4_sitl_default
```

Успех: `[100%] Built target px4`, бинарь `build/px4_sitl_default/bin/px4`.

### A.4. Запустить SITL + Gazebo и проверить взлёт

Запуск — через скрипт проекта (он один процесс, не ROS2-нода, поэтому shell, а не launch):

```bash
# из корня проекта drone-simulator-tracking
scripts/run_px4_sitl.sh            # модель gz_x500_mono_cam, с GUI Gazebo
# варианты:
HEADLESS=1 scripts/run_px4_sitl.sh # без GUI (CI/слабая машина)
GPU=nvidia scripts/run_px4_sitl.sh # форс рендера на NVIDIA (гибридная графика — см. ниже)
```

Скрипт делает `cd ~/src/PX4-Autopilot && make px4_sitl gz_x500_mono_cam`. Путь к PX4 и
модель переопределяются переменными `PX4_AUTOPILOT_DIR` / `PX4_MODEL` (см. шапку скрипта).

**Проверка go/no-go** — в появившемся приглашении `pxh>`:

```
pxh> commander takeoff
```

Дрон должен взлететь (в GUI видно физически; в headless — по телеметрии):

```
pxh> listener vehicle_local_position 1
```

Признак успеха: в логе `Armed by internal command` → `Takeoff detected`, а в выводе
`listener` поле `z` уходит в минус (NED: `z<0` = вверх), напр. `z: -1.96` ≈ 2 м набора.
Остановить: `pxh> shutdown` (корректно гасит PX4 и Gazebo). При зависших хвостах —
`scripts/stop_sim.sh` (см. «Остановка» в чек-листе ниже).

> Проверено 2026-06-11 (headless): дошёл до `Ready for takeoff!`, `uxrce_dds_client`
> поднял UDP-агента на `127.0.0.1:8888`, взлёт состоялся (`z ≈ −1.96`).

---

## Инкремент B — uXRCE-DDS мост (план §8.7) ✅

Цель: uORB-топики PX4 видны и читаются в ROS2 через Micro-XRCE-DDS-Agent.

### B.1. Добавить `px4_msgs` в workspace

`px4_msgs` — это ROS2-определения сообщений PX4 (генерируются под версию автопилота).
Без него `ros2 topic echo` не разберёт сообщения. Это **внешний код**: клонируем в `src/`,
но в git проекта не коммитим (он в `.gitignore`). Ветка — под релиз PX4 (`release/1.15`):

```bash
# из корня проекта drone-simulator-tracking
git clone -b release/1.15 --depth 1 \
  https://github.com/PX4/px4_msgs.git src/px4_msgs
```

(Зафиксированный commit: `a1045ec`. Версия моста — Micro-XRCE-DDS-Agent `v2.4.3`,
стоит системно из Фазы 0.)

### B.2. Собрать workspace

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
colcon build            # px4_msgs генерирует ~2 мин (все uORB-сообщения)
```

Успех: `Summary: 5 packages finished`. Сборка `px4_msgs` — самая долгая.

### B.3. Запустить агент и проверить топики

Агент — это вторая половина моста (первую, `uxrce_dds_client`, PX4 SITL поднимает сам).
В **отдельном терминале**:

```bash
scripts/run_xrce_agent.sh          # MicroXRCEAgent udp4 -p 8888
```

В **третьем терминале** — SITL (`scripts/run_px4_sitl.sh`, см. Инкремент A). Порядок
запуска агента и SITL не важен — клиент переподключится.

**Проверка go/no-go** — в терминале с настроенным ROS2-окружением:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash          # из корня проекта; даёт px4_msgs
source .venv/bin/activate
ros2 topic list | grep '^/fmu/'                       # должны появиться /fmu/in/* и /fmu/out/*
ros2 topic hz   /fmu/out/vehicle_local_position       # стабильный поток
ros2 topic echo --once /fmu/out/vehicle_local_position # один сэмпл телеметрии
```

Признак успеха: список содержит десятки `/fmu/*` топиков; `hz` показывает стабильную
частоту; `echo` печатает реальные `x/y/z/v*`. Если `/fmu/*` пусто — не запущен агент
(см. §9 gotchas) или не сделан `source install/setup.bash`.

> Проверено 2026-06-11 (headless): **43** `/fmu/*` топика; `vehicle_local_position` идёт
> на **100 Гц**; `echo` отдаёт живые значения. Агент логирует `publisher created` /
> `datawriter created` — клиент PX4 подключился.

## Инкремент C — камера через ros_gz_bridge (план §8.8, гейт Фазы 2) ✅

Цель (go/no-go всей Фазы 2): кадр из камеры дрона приходит в ROS2 как `/camera/image`.

### C.1. Что уже сделано в коде

- `configs/simulator/camera_bridge.yaml` — правила `ros_gz_bridge`: gz `/camera`
  (`gz.msgs.Image`) → ROS `/camera/image` (`sensor_msgs/msg/Image`), плюс `/camera_info`.
- `src/drone_simulator/launch/sim.launch.py` — поднимает **агент** (`MicroXRCEAgent`) и
  **камера-мост** (`ros_gz_bridge parameter_bridge`) одной командой.
- `setup.py`/`package.xml` пакета `drone_simulator` ставят launch + копию конфига в share
  и объявляют зависимость `ros_gz_bridge`.

Имена/тип gz-топика подтверждены вживую (`gz topic -l` → `/camera`, `/camera_info`;
`gz topic -i -t /camera` → `gz.msgs.Image`, 640×480 RGB, 30 Гц). Камера рендерится и в
**headless**-режиме (GUI не обязателен).

### C.2. Пересобрать (после правок конфига/launch)

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
colcon build --packages-select drone_simulator
```

> Конфиг ставится копией в `install/.../share/drone_simulator/config/`. Источник истины —
> `configs/simulator/camera_bridge.yaml`; после его правки нужен `colcon build`, **либо**
> запуск с `bridge_config:=<абсолютный путь к configs/...>` (см. аргумент в sim.launch.py).

### C.3. Запустить демо и проверить кадр

**Терминал 1** — PX4 SITL + Gazebo (модель с камерой):
```bash
scripts/run_px4_sitl.sh
# варианты:
HEADLESS=1 scripts/run_px4_sitl.sh # без GUI
GPU=nvidia scripts/run_px4_sitl.sh # форс рендера на NVIDIA
```
**Терминал 2** — ROS2-сторона (агент + камера-мост):
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && source .venv/bin/activate
ros2 launch drone_simulator sim.launch.py
```

**Проверка go/no-go** (терминал 3, окружение как в B.3):
```bash
ros2 topic hz /camera/image          # ~30 Гц
ros2 run rqt_image_view rqt_image_view /camera/image   # визуально видно кадр (нужен GUI)
```

Признак успеха: `hz` ≈ 30; в `rqt_image_view` — картинка из камеры дрона; в логе моста —
`Creating GZ->ROS Bridge: [/camera (gz.msgs.Image) -> /camera/image ...]`.

> Проверено 2026-06-11 (headless): `/camera/image` идёт на **~30 Гц**, `640×480`, `encoding:
> rgb8`, `frame_id: x500_mono_cam_0/mono_cam/base_link/imager`. Сохранённый кадр —
> `outputs/videos/phase2_camera_sample.png` (пустой мир `default.sdf`: видна линия горизонта,
> объектов-целей нет — они появятся в Фазе 3). Кадр сохранялся без GUI через `cv_bridge`.

---

## Ручная проверка и визуализация (чек-лист)

Раздел для проверки результата Фазы 2 **глазами** (выше — про установку; здесь — про
«пощупать»). Нужен дисплей; GUI-инструменты `rqt_image_view`/`rviz2` ставятся с
`ros-humble-desktop` (Фаза 0).

### Подготовка — в каждом терминале (обязательно)

Каждый **новый** терминал стартует с «голым» окружением — слои ROS2 не наследуются. Простое
безопасное правило: **в каждом терминале** из корня проекта выполнить три `source` (порядок
не важен, лишним ни в одном не будет):
```bash
cd ~/Programming/projects/ai_drones/project_2/drone-simulator-tracking
source /opt/ros/humble/setup.bash    # ROS2 (иначе: ros2: command not found)
source install/setup.bash            # наши пакеты + px4_msgs (иначе: Package '…' not found)
source .venv/bin/activate            # python-зависимости (в терминале PyCharm уже активен)
```
Маркер готовности: `echo $ROS_DISTRO` → `humble`; `ros2 pkg prefix drone_simulator` → путь
внутри `…/install/…` (а не ошибка).

Что какому терминалу реально нужно (если хочется по минимуму):

| Терминал | `/opt/ros/humble` | `install/setup.bash` | `.venv` | Почему |
|---|---|---|---|---|
| 1 — `run_px4_sitl.sh` (SITL) | нет | нет | нет | скрипт сам зовёт `make` PX4; наши слои ему не нужны (но три `source` не мешают) |
| 2 — `ros2 launch …` | **да** | **да** | не обяз. | без оверлея — `Package 'drone_simulator' not found` |
| 3 — `ros2 topic …` | **да** | **да** | не обяз. | `install/setup.bash` даёт `px4_msgs`, иначе `echo /fmu/*` не разберёт типы |

> 💡 Самая частая ошибка — забыть `source install/setup.bash` в терминалах 2/3 (даёт
> `Package … not found` или пустой `/fmu/*`). Поэтому проще не экономить и сорсить все три
> слоя везде.

### Терминал 1 — PX4 SITL + Gazebo (с GUI, видно дрон)
```bash
scripts/run_px4_sitl.sh
# варианты:
HEADLESS=1 scripts/run_px4_sitl.sh # без GUI
GPU=nvidia scripts/run_px4_sitl.sh # форс рендера на NVIDIA
```
Откроется окно Gazebo с дроном `x500_mono_cam`. После `Ready for takeoff!` в `pxh>`:
```
pxh> commander takeoff      # дрон физически взлетает в окне Gazebo (~2.5 м) — проверка A
pxh> listener vehicle_local_position 1   # вывести сообщение топика 1 раз; смотрим поле z (высота, NED: z<0 = вверх)
pxh> commander land         # посадка
pxh> shutdown               # стоп
```

### Терминал 2 — ROS2-сторона (агент + камера-мост)
Сначала окружение (см. «Подготовка» — нужны `/opt/ros/humble` + `install/setup.bash`):
```bash
ros2 launch drone_simulator sim.launch.py
```
В логе ищи `Creating GZ->ROS Bridge: [/camera (gz.msgs.Image) -> /camera/image ...]`.

### Терминал 3 — числовые проверки
Окружение — как в «Подготовка» (`install/setup.bash` обязателен, иначе `echo` не разберёт типы):
```bash
# B — телеметрия PX4 в ROS2:
ros2 topic list | grep '^/fmu/'                        # десятки топиков (было 43)
ros2 topic hz   /fmu/out/vehicle_local_position        # ~100 Гц
ros2 topic echo --once /fmu/out/vehicle_local_position # живые x/y/z

# C — кадр камеры в ROS2:
ros2 topic hz   /camera/image                          # ~30 Гц
ros2 topic echo --once /camera/image --field encoding  # rgb8
```

### Визуализация камеры (что «видит» дрон)
```bash
ros2 run rqt_image_view rqt_image_view /camera/image
```
Окно с живым кадром 640×480. В пустом мире `default.sdf` — линия горизонта; подвигай дрон
(`commander takeoff` и далее) — картинка меняется в реальном времени. Сохранённый одиночный
кадр для сравнения — `outputs/videos/phase2_camera_sample.png`. Альтернатива — `rviz2` →
Add → By topic → `/camera/image` → Image (для камеры `rqt_image_view` проще).

### Что подтверждает каждый шаг

| Видишь | Значит работает |
|---|---|
| Дрон взлетает в окне Gazebo | A: PX4 SITL + физика + airframe |
| `/fmu/*` в списке, `hz` ~100 | B: мост uXRCE-DDS (PX4 → ROS2) |
| Картинка в `rqt_image_view`, `hz` ~30 | C: камера через ros_gz_bridge (Gazebo → ROS2) — **гейт Фазы 2** |

### Нюансы
- Порядок: сначала Терминал 1 (SITL), потом 2 (мост). Порядок агента и SITL некритичен —
  клиент переподключится, но логичнее сперва поднять симулятор.
- `/fmu/*` пусто → не запущен `sim.launch.py` (нет агента) или забыт `source install/setup.bash`.
- Без окон (по SSH): `HEADLESS=1 scripts/run_px4_sitl.sh`; камера публикуется, но GUI смысла не имеет.

### Гибридная графика (камера молчит в GUI) — device-specific

**Симптом:** в headless `/camera/image` идёт ~30 Гц, а с GUI `ros2 topic hz /camera/image`
показывает 0 кадров (хотя `/fmu/*` и взлёт работают). В логе SITL —
`libEGL warning: egl: failed to create dri2 screen`.

**Причина:** на ноутбуке с гибридной графикой (Intel + NVIDIA) в GUI-режиме окно Gazebo
забирает GPU, и off-screen рендер камеры в gz-сервере голодает. Сам пайплайн исправен —
это подтверждается тем, что в `HEADLESS=1` камера идёт 30 Гц.

**Решение (opt-in):** форсить рендер на дискретную NVIDIA — флаг `GPU=nvidia` у скрипта SITL:
```bash
GPU=nvidia scripts/run_px4_sitl.sh        # Терминал 1
```
Флаг выставляет `__NV_PRIME_RENDER_OFFLOAD=1`, `__GLX_VENDOR_LIBRARY_NAME=nvidia` и
`__EGL_VENDOR_LIBRARY_FILENAMES=…/10_nvidia.json`. По умолчанию (без флага) — системный
выбор GPU, ничего не навязывается. Переменные нужны **только** Терминалу 1 (рендерит
Gazebo); Терминалу 2 (агент + мост) — нет, он ничего не рендерит.

> Значения device-specific: на другой машине путь к EGL-вендору может отличаться — задать
> через `NVIDIA_EGL_JSON=<путь> GPU=nvidia …`, или подобрать свой способ выбора GPU. Проверено
> на этой машине (Intel UHD + RTX 3050 Ti, 2026-06-12): `GPU=nvidia` → камера 30 Гц с GUI.

### Остановка (важно — иначе перегрев)

`Ctrl-C` в `ros2 launch` шлёт сигнал группе launch, но сервер Gazebo (`gz sim -s`, его PX4
поднимает отдельно) часто **переживает Ctrl-C** и остаётся «осиротевшим» — крутит физику
вхолостую и греет CPU/GPU. Надёжный порядок:

1. Терминал 1: `pxh> shutdown` — корректно гасит PX4 **и** связанный Gazebo.
2. Терминал 2: `Ctrl-C`.
3. Контроль/добивание хвостов одной командой:
   ```bash
   scripts/stop_sim.sh
   ```
   Скрипт идемпотентен: гасит `px4`/`gz sim`/`MicroXRCEAgent`/`parameter_bridge`
   (сначала SIGTERM, через 2 с — SIGKILL уцелевшим), на чистой системе просто пишет
   «гасить нечего». Ручная проверка, если нужно: `pgrep -af "gz sim|bin/px4|MicroXRCEAgent"`.
