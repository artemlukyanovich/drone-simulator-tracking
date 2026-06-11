#!/usr/bin/env bash
# launch_pycharm.sh — запускает PyCharm с полностью засорсенным ROS2-окружением.
#
# Зачем: PyCharm, запущенный из меню/иконки, не видит `rclpy` (и др. ROS2-пакеты) —
# они подключаются не через site-packages, а через PYTHONPATH, который выставляет
# `source /opt/ros/humble/setup.bash`. Этот скрипт сорсит окружение (ROS2 → workspace
# → venv) и стартует IDE, чтобы её анализатор и run-конфигурации видели весь стек.
# Источник истины по окружению — docs/project_plan.md §4 (Р7) и configs/simulator/stack.md.
#
# Использование:
#   scripts/launch_pycharm.sh              # открыть текущий workspace в PyCharm
#   scripts/launch_pycharm.sh путь/к/файлу # открыть конкретный файл/папку
#
# PyCharm стартует в фоне (detached): можно закрыть терминал, IDE продолжит работать.
#
# TODO (позже): прописать этот скрипт в Exec= файла
#   ~/.local/share/applications/jetbrains-pycharm.desktop,
# чтобы и иконка в меню запускала PyCharm с засорсенным окружением, а не «голым».

set -euo pipefail

# --- Корень workspace (= папка над scripts/), независимо от текущего cwd ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# --- 1. Системный ROS2 (даёт rclpy и PYTHONPATH) ---
ROS_SETUP="/opt/ros/humble/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ERROR: не найден ${ROS_SETUP} — установлен ли ROS2 Humble?" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "${ROS_SETUP}"

# --- 2. Сборка workspace (даёт пакеты drone_*); опционально — если уже собран ---
WS_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
if [[ -f "${WS_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${WS_SETUP}"
else
  echo "NOTE: ${WS_SETUP} нет — сначала 'colcon build'. Пакеты drone_* пока не в PYTHONPATH." >&2
fi

# --- 3. venv проекта (даёт torch/ultralytics поверх системного rclpy) ---
VENV_ACTIVATE="${WORKSPACE_ROOT}/.venv/bin/activate"
if [[ -f "${VENV_ACTIVATE}" ]]; then
  # shellcheck disable=SC1090
  source "${VENV_ACTIVATE}"
else
  echo "NOTE: ${VENV_ACTIVATE} нет — запусти scripts/setup_venv.sh." >&2
fi

# --- 4. Найти лаунчер PyCharm (Toolbox меняет суффикс apps/pycharm-*-N при обновлениях) ---
PYCHARM_BIN=""
# 4a. Самый свежий из JetBrains Toolbox.
for cand in $(ls -dt "${HOME}/.local/share/JetBrains/Toolbox/apps/"*/bin/pycharm 2>/dev/null); do
  PYCHARM_BIN="${cand}"; break
done
# 4b. Фолбэк: вытащить Exec= из .desktop.
if [[ -z "${PYCHARM_BIN}" ]]; then
  DESKTOP="${HOME}/.local/share/applications/jetbrains-pycharm.desktop"
  if [[ -f "${DESKTOP}" ]]; then
    PYCHARM_BIN="$(grep -m1 '^Exec=' "${DESKTOP}" | sed -E 's/^Exec=//; s/ ?%[a-zA-Z].*$//; s/^"//; s/"$//')"
  fi
fi
# 4c. Фолбэк: что-то в PATH.
if [[ -z "${PYCHARM_BIN}" ]]; then
  PYCHARM_BIN="$(command -v pycharm pycharm.sh 2>/dev/null | head -1 || true)"
fi
if [[ -z "${PYCHARM_BIN}" || ! -x "${PYCHARM_BIN}" ]]; then
  echo "ERROR: не нашёл исполняемый файл PyCharm. Укажи путь вручную в скрипте." >&2
  exit 1
fi

# --- 5. Что открыть: переданный аргумент или сам workspace ---
TARGET="${1:-${WORKSPACE_ROOT}}"

echo "ROS_DISTRO=${ROS_DISTRO:-?}  |  PyCharm: ${PYCHARM_BIN}"
echo "Открываю: ${TARGET}"

# Detached: IDE переживёт закрытие терминала; логи — в /tmp.
nohup "${PYCHARM_BIN}" "${TARGET}" >/tmp/pycharm_drone.log 2>&1 &
disown
