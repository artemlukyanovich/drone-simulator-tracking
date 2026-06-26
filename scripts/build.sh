#!/usr/bin/env bash
#
# build.sh — правильная сборка colcon-workspace ДЛЯ ЭТОГО ПРОЕКТА.
#
# Зачем отдельный скрипт, а не голый `colcon build`:
#   Ноды Фазы 3 (detector_node, follower_node) импортируют pip-зависимости из .venv
#   (torch/ultralytics). ament_python штампует в сгенерированный entry-point shebang
#   того python'а, под которым шла сборка. Системный `colcon` — это скрипт с shebang
#   /usr/bin/python3, поэтому обычный `colcon build` даёт ноды с системным python,
#   который НЕ видит torch (он только в .venv) → `ModuleNotFoundError: No module
#   named 'torch'` при запуске.
#
#   Решение: собирать colcon ПОД venv-python. Тогда shebang узлов указывает на
#   .venv/bin/python, который видит и torch (свой site-packages), и системный rclpy
#   (через PYTHONPATH от /opt/ros, venv создан с --system-site-packages).
#
# Использование (из корня проекта):
#   scripts/build.sh                              # собрать всё
#   scripts/build.sh --packages-select drone_perception   # как у colcon, аргументы прокидываются
#
# Подробности ловушки — docs/phase3_setup.md §12.
#
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

if [[ ! -x .venv/bin/python ]]; then
  echo "✗ .venv не найден (.venv/bin/python). Создай: scripts/setup_venv.sh" >&2
  exit 1
fi

# ROS2 нужен для зависимостей сборки (rosidl, ament, message-пакеты).
# setup.bash ссылается на необъявленные переменные → временно снимаем nounset.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

COLCON="$(command -v colcon || true)"
if [[ -z "${COLCON}" ]]; then
  echo "✗ colcon не найден в PATH (нужен системный apt-colcon)." >&2
  exit 1
fi

echo "→ colcon build под venv-python (${PROJECT_DIR}/.venv/bin/python)"
exec .venv/bin/python "${COLCON}" build "$@"
