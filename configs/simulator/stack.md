# Зафиксированный стек симулятора

Рабочая тройка версий проекта (источник истины по решению — `docs/project_plan.md` §4, Р7).

| Компонент | Версия | Примечание |
|---|---|---|
| База ОС | Ubuntu 22.04 (Linux Mint 21.3) | apt-репо ROS2/Gazebo добавлять с codename `jammy` (Mint отдаёт `virginia`) |
| ROS2 | Humble Hawksbill (LTS) | Python 3.10 |
| Gazebo | Harmonic (gz-sim8, LTS) | мост к ROS2 — пакет `ros-humble-ros-gzharmonic` |
| PX4 | v1.15.x | поддерживает Humble + uXRCE-DDS + Harmonic |
| Мост PX4↔ROS2 | Micro-XRCE-DDS-Agent + `px4_msgs` | ветку `px4_msgs` брать под выбранный релиз PX4 |
| Python | 3.10 (системный) | venv создаём с `--system-site-packages` |
| GPU | NVIDIA RTX 3050 Ti, драйвер 580 (CUDA 12.x) | для YOLO |

## Окружение

Нативная установка, без Docker (на этапе локальной разработки):
- ROS2 Humble — системно через apt (`/opt/ros/humble`).
- venv с `--system-site-packages`, чтобы ноды видели и системный `rclpy`, и pip-зависимости.
- Порядок активации: `source /opt/ros/humble/setup.bash` → активировать venv.

## TODO при установке
- [ ] Подтвердить связку PX4 ↔ Gazebo (Harmonic vs Garden) по docs.px4.io.
- [ ] Зафиксировать точные теги/коммиты PX4-Autopilot и px4_msgs после успешной сборки.
