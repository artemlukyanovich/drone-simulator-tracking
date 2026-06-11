# Зафиксированный стек симулятора

Рабочая тройка версий проекта (источник истины по решению — `docs/project_plan.md` §4, Р7).

| Компонент | Версия | Примечание |
|---|---|---|
| База ОС | Ubuntu 22.04 (Linux Mint 21.3) | apt-репо ROS2/Gazebo добавлять с codename `jammy` (Mint отдаёт `virginia`) |
| ROS2 | Humble Hawksbill (LTS) | Python 3.10 |
| Gazebo | Harmonic (gz-sim8, LTS) | мост к ROS2 — пакет `ros-humble-ros-gzharmonic` |
| PX4 | v1.15.x | поддерживает Humble + uXRCE-DDS + Harmonic |
| Мост PX4↔ROS2 | Micro-XRCE-DDS-Agent `v2.4.3` + `px4_msgs` | агента собираем из исходников (нет apt-пакета, snap на Mint заблокирован); ветку `px4_msgs` — под релиз PX4 |
| Python | 3.10 (системный) | venv создаём с `--system-site-packages` |
| GPU | NVIDIA RTX 3050 Ti, драйвер 580 (CUDA 12.x) | для YOLO |

## Окружение

Нативная установка, без Docker (на этапе локальной разработки):
- ROS2 Humble — системно через apt (`/opt/ros/humble`).
- venv с `--system-site-packages`, чтобы ноды видели и системный `rclpy`, и pip-зависимости.
- Создание/обновление venv — скриптом `scripts/setup_venv.sh` (поверх системного python 3.10,
  ставит зависимости из `requirements.txt`). conda не используем — он тянет свой интерпретатор/ABI
  и ломает линковку против системного ROS2.
- pip-зависимости (`torch`, `ultralytics`, `opencv-python`) — в `requirements.txt`.
  **numpy зафиксирован `<2`**: ROS2 Humble / cv_bridge собраны под numpy 1.x.
- Активация для работы — нужны **оба слоя**, но **порядок между ними не важен**
  (проверено 2026-06-11: `venv→ROS2` и `ROS2→venv` дают идентичный `sys.path` — тот же
  python из venv, numpy 1.26.4 из venv, rclpy из `/opt/ros/humble`). Минимальный набор:
  ```bash
  source /opt/ros/humble/setup.bash   # rclpy + PYTHONPATH/LD_LIBRARY_PATH/AMENT_PREFIX_PATH
  source install/setup.bash           # пакеты drone_* (после colcon build)
  source .venv/bin/activate           # torch/ultralytics; в терминале PyCharm уже активен
  ```
  Маркер готовности: `echo $ROS_DISTRO` → `humble`. Единственное жёсткое «нельзя» —
  **conda** (свой интерпретатор/ABI ломает линковку против системного ROS2); обычный
  `venv --system-site-packages` безопасен в любом порядке.
- `source install/setup.bash` пересорсить **обязательно** в новом терминале и при изменении
  состава пакетов (добавил/переименовал). При инкрементальной пересборке того же набора в
  том же терминале — не обязательно (env указывает на те же пути), но привычка полезна.

## Проверено на машине (Фаза 0, 2026-06-10)

Системный + venv-слой собраны и проверены на интеграцию:

| Компонент | Установленная версия |
|---|---|
| Gazebo Sim | 8.13.0 (Harmonic) |
| Micro-XRCE-DDS-Agent | v2.4.3 (собран из исходников в `~/src/`) |
| torch | 2.12.0 — колёса **CUDA 13** (`nvidia-*-cu13`), `cuda.is_available() == True` на драйвере 580 |
| numpy | 1.26.4 (в venv, затеняет системный 1.21.5) |
| ultralytics | 8.4.63 |
| opencv-python | 4.11.0.86 |

> torch подтянул CUDA 13, а не cu12.1 (драйвер 580 — минимум для CUDA 13.0). Если переустанавливать
> под cu12.1 — команда в `requirements.txt`. Пока работает на cu13 — не трогаем.

## TODO при установке
- [ ] Подтвердить связку PX4 ↔ Gazebo (Harmonic vs Garden) по docs.px4.io.
- [ ] Подтвердить версию Micro-XRCE-DDS-Agent (`v2.4.3`) под выбранный релиз PX4 по docs.px4.io.
- [ ] Зафиксировать точные теги/коммиты PX4-Autopilot и px4_msgs после успешной сборки.
