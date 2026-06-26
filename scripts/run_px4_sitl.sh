#!/usr/bin/env bash
#
# run_px4_sitl.sh — запуск PX4 SITL + Gazebo Harmonic с моделью x500_mono_cam.
#
# PX4 SITL — это отдельный процесс автопилота (не ROS2-нода), поэтому он живёт
# в shell-скрипте, а не в launch-файле (см. docs/project_plan.md §8, Фаза 2).
# Цель make-а `gz_<model>` одновременно собирает прошивку (если нужно) и
# поднимает Gazebo + PX4. uxrce_dds_client стартует внутри SITL автоматически и
# ждёт агента на 127.0.0.1:8888 (агент — scripts/run_xrce_agent.sh, Фаза 2 §7).
#
# Переменные окружения:
#   PX4_AUTOPILOT_DIR  путь к PX4-Autopilot (по умолчанию ~/src/PX4-Autopilot)
#   PX4_MODEL          модель Gazebo (по умолчанию gz_x500_mono_cam)
#   HEADLESS=1         запуск Gazebo без GUI (для CI/слабой машины)
#   GPU=nvidia         форсить рендер Gazebo на дискретную NVIDIA (гибридная графика).
#                      По умолчанию пусто — системный выбор GPU. См. ниже и
#                      docs/phase2_setup.md («Гибридная графика»).
#   WORLD=<путь.sdf>   standalone-режим: МЫ сами поднимаем Gazebo с этим миром, а
#                      PX4 к нему подключается (PX4_GZ_STANDALONE). Нужно для своего
#                      мира с целью (Фаза 3), не правя PX4. <world> внутри SDF обязан
#                      называться "default" (см. docs/phase3_setup.md, инкремент 1).
#                      Без WORLD — прежнее поведение: PX4 сам поднимает default-мир.
#
set -euo pipefail

PX4_DIR="${PX4_AUTOPILOT_DIR:-${HOME}/src/PX4-Autopilot}"
PX4_MODEL="${PX4_MODEL:-gz_x500_mono_cam}"
export HEADLESS="${HEADLESS:-0}"
WORLD="${WORLD:-}"

if [[ ! -d "${PX4_DIR}" ]]; then
  echo "✗ PX4-Autopilot не найден: ${PX4_DIR}" >&2
  echo "  Склонируй фиксированный тег (см. configs/simulator/stack.md):" >&2
  echo "    git clone -b v1.15.4 --recurse-submodules \\" >&2
  echo "      https://github.com/PX4/PX4-Autopilot.git ${PX4_DIR}" >&2
  exit 1
fi

# Выбор GPU для рендера сенсоров Gazebo. На ноутбуках с гибридной графикой
# (Intel + NVIDIA) в GUI-режиме рендер камеры может голодать — в логе
# `libEGL warning: egl: failed to create dri2 screen`, и `/camera/image` молчит.
# GPU=nvidia форсит рендер на дискретную карту (PRIME offload + EGL vendor).
# Значение и путь к EGL-вендору device-specific, поэтому это opt-in, не дефолт.
NVIDIA_EGL_JSON="${NVIDIA_EGL_JSON:-/usr/share/glvnd/egl_vendor.d/10_nvidia.json}"
case "${GPU:-}" in
  nvidia)
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    if [[ -f "${NVIDIA_EGL_JSON}" ]]; then
      export __EGL_VENDOR_LIBRARY_FILENAMES="${NVIDIA_EGL_JSON}"
    else
      echo "⚠ GPU=nvidia: EGL-вендор не найден (${NVIDIA_EGL_JSON}); задай NVIDIA_EGL_JSON=<путь>." >&2
    fi
    echo "→ GPU=nvidia: форсирую рендер Gazebo на NVIDIA (PRIME offload + EGL vendor)"
    ;;
  ""|system|default)
    : # системный выбор GPU по умолчанию — ничего не трогаем
    ;;
  *)
    echo "⚠ Неизвестное GPU='${GPU}' (ожидается nvidia|system). Использую системный выбор." >&2
    ;;
esac

# ── Standalone-режим: свой мир (Фаза 3) ──────────────────────────────────────
# PX4 не отдаёт мир под подмену через env (перетирает PX4_GZ_WORLDS сорсингом
# gz_env.sh), поэтому штатный путь для своего мира — поднять Gazebo самим, а PX4
# запустить с PX4_GZ_STANDALONE=1: он не стартует свой gz, а цепляется к нашему.
if [[ -n "${WORLD}" ]]; then
  WORLD="$(readlink -f "${WORLD}")"
  if [[ ! -f "${WORLD}" ]]; then
    echo "✗ WORLD не найден: ${WORLD}" >&2
    exit 1
  fi
  WORLD_NAME="$(grep -oP '<world name="\K[^"]+' "${WORLD}" | head -1)"

  # Сервер Gazebo должен резолвить модели PX4 (когда gz_bridge спавнит
  # x500_mono_cam) — кладём их в GZ_SIM_RESOURCE_PATH. Fuel-модель цели сервер
  # докачает сам по URI из SDF.
  export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds"

  echo "→ standalone Gazebo: world=${WORLD_NAME} (${WORLD})"
  gz sim -r -v1 -s "${WORLD}" &
  GZ_SERVER_PID=$!
  if [[ "${HEADLESS}" != "1" ]]; then
    gz sim -g &
    GZ_GUI_PID=$!
  fi

  # PX4 в standalone не владеет Gazebo — гасим сервер/GUI при выходе скрипта.
  cleanup() {
    [[ -n "${GZ_SERVER_PID:-}" ]] && kill "${GZ_SERVER_PID}" 2>/dev/null || true
    [[ -n "${GZ_GUI_PID:-}" ]] && kill "${GZ_GUI_PID}" 2>/dev/null || true
  }
  trap cleanup EXIT INT TERM

  # Ждём, пока сервер поднимет мир (clock-топик): иначе gz_bridge PX4 не к чему
  # цеплять. Первый запуск дольше — Gazebo докачивает Fuel-модель человека.
  echo "  ожидаю /world/${WORLD_NAME}/clock (первый запуск дольше — качается Fuel-модель)…"
  for _ in $(seq 1 240); do
    if gz topic -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/clock"; then
      echo "  ✓ мир поднят"
      break
    fi
    sleep 0.5
  done

  echo "→ PX4 SITL (standalone attach): dir=${PX4_DIR}  model=${PX4_MODEL}  HEADLESS=${HEADLESS}  GPU=${GPU:-system}"
  cd "${PX4_DIR}"
  PX4_GZ_STANDALONE=1 make px4_sitl "${PX4_MODEL}"
  exit 0
fi

# ── Штатный режим (Фаза 2): PX4 сам поднимает default-мир ─────────────────────
echo "→ PX4 SITL: dir=${PX4_DIR}  model=${PX4_MODEL}  HEADLESS=${HEADLESS}  GPU=${GPU:-system}"
cd "${PX4_DIR}"
exec make px4_sitl "${PX4_MODEL}"
