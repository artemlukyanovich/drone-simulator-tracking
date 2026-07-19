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

# --- Прогрев весов ReID (Фаза 4, M2 / Ф4-17) ---
# ЗАЧЕМ. ObjectEmbedder грузит OpenCLIP СИНХРОННО в конструкторе ноды. Если весов
# (~350 МБ) нет в кэше, они качаются прямо на старте detector_node — а follower к тому
# моменту уже армится и взлетает. Снаружи это выглядит поломкой: дрон крутится в SEARCH,
# потому что детектор ещё не отдал ни одной цели. Скачиваем заранее, здесь.
# Имя модели берём из конфига перцепции, чтобы не дублировать значения (§7 CLAUDE.md).
# Не критично: нет сети — просто предупреждаем, ReID сам отключится и прогон пойдёт без него.
if [[ "${PREWARM_REID:-1}" == "1" ]]; then
  echo "→ Прогреваю веса ReID (разово; при наличии в кэше — мгновенно)"
  python - "${REPO_ROOT}" <<'PY' || echo "⚠️  Прогрев не удался (нет сети?). ReID отключится сам, слежение будет работать без перезахвата."
import sys, pathlib, yaml
cfg = yaml.safe_load((pathlib.Path(sys.argv[1]) / 'configs/perception/detector.yaml').read_text())
reid = cfg['detector_node']['ros__parameters'].get('reid', {})
if not reid.get('enabled', True):
    print('  reid.enabled=false — прогрев не нужен')
    sys.exit(0)
name, tag = reid.get('model_name', 'ViT-B-32'), reid.get('pretrained', 'laion2b_s34b_b79k')
import open_clip
open_clip.create_model_and_transforms(name, pretrained=tag, device='cpu')
print(f'  ✓ веса {name}/{tag} на месте')
PY
fi

cat <<EOF

✅ Готово.

Порядок активации окружения для работы (важен!):
    source /opt/ros/humble/setup.bash      # сперва ROS2
    source ${VENV_DIR}/bin/activate        # затем venv поверх

Проверка, что венв видит системный rclpy:
    python -c "import rclpy; print('rclpy OK')"
EOF
