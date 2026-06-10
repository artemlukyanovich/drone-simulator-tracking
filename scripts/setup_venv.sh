#!/usr/bin/env bash
#
# setup_venv.sh — создаёт/обновляет venv проекта.
#
# Идея (см. docs/project_plan.md Р4, Р7):
#   venv создаётся поверх СИСТЕМНОГО python3 (3.10, штатный под ROS2 Humble)
#   с флагом --system-site-packages, чтобы ROS2-ноды видели И системный rclpy/
#   ros_gz/cv_bridge (из apt), И pip-зависимости проекта (torch/ultralytics).
#   conda здесь НЕ годится: он тянет свой интерпретатор/ABI и ломает линковку
#   против системного ROS2.
#
# Идемпотентен: повторный запуск не пересоздаёт venv, а только доустанавливает
# зависимости из requirements.txt.
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${REPO_ROOT}/.venv"

# --- Проверка версии системного python (стек завязан на 3.10 под Humble) ---
PY_VERSION="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
if [[ "${PY_VERSION}" != "3.10" ]]; then
  echo "⚠️  Системный python3 = ${PY_VERSION}, а стек завязан на 3.10 (ROS2 Humble)." >&2
  echo "    Создавать venv под другой минорной версией опасно — прерываюсь." >&2
  exit 1
fi

# --- Создание venv (если ещё нет) ---
if [[ ! -d "${VENV_DIR}" ]]; then
  echo "→ Создаю venv: ${VENV_DIR}  (--system-site-packages)"
  python3 -m venv --system-site-packages "${VENV_DIR}"
else
  echo "→ venv уже существует: ${VENV_DIR}  (пропускаю создание)"
fi

# --- Установка/обновление зависимостей ---
# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
python -m pip install --upgrade pip

if [[ -f "${REPO_ROOT}/requirements.txt" ]]; then
  echo "→ Устанавливаю зависимости из requirements.txt"
  python -m pip install -r "${REPO_ROOT}/requirements.txt"
else
  echo "→ requirements.txt не найден — пропускаю установку зависимостей"
fi

cat <<EOF

✅ Готово.

Порядок активации окружения для работы (важен!):
    source /opt/ros/humble/setup.bash      # сперва ROS2
    source ${VENV_DIR}/bin/activate        # затем venv поверх

Проверка, что венв видит системный rclpy:
    python -c "import rclpy; print('rclpy OK')"
EOF
