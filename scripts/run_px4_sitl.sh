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
#
set -euo pipefail

PX4_DIR="${PX4_AUTOPILOT_DIR:-${HOME}/src/PX4-Autopilot}"
PX4_MODEL="${PX4_MODEL:-gz_x500_mono_cam}"
export HEADLESS="${HEADLESS:-0}"

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

echo "→ PX4 SITL: dir=${PX4_DIR}  model=${PX4_MODEL}  HEADLESS=${HEADLESS}  GPU=${GPU:-system}"
cd "${PX4_DIR}"
exec make px4_sitl "${PX4_MODEL}"
