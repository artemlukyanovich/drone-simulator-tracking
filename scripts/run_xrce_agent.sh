#!/usr/bin/env bash
#
# run_xrce_agent.sh — мост PX4 ↔ ROS2 (Micro-XRCE-DDS-Agent).
#
# PX4 SITL внутри себя запускает uxrce_dds_client, который коннектится на UDP 8888.
# Этот агент — вторая половина моста: он отдаёт uORB-топики PX4 в DDS/ROS2.
# Без запущенного агента PX4-топики НЕ появятся в `ros2 topic list` (см. §9 gotchas).
#
# Порядок запуска роли не играет: агент можно поднять до или после SITL — клиент
# переподключится. Агент должен жить всё время, пока работает SITL.
#
# Переменная окружения:
#   XRCE_PORT  UDP-порт агента (по умолчанию 8888 — совпадает с дефолтом PX4 SITL)
#
set -euo pipefail

XRCE_PORT="${XRCE_PORT:-8888}"

if ! command -v MicroXRCEAgent > /dev/null 2>&1; then
  echo "✗ MicroXRCEAgent не найден в PATH." >&2
  echo "  Поставь системный слой: scripts/install_system_deps.sh" >&2
  exit 1
fi

echo "→ Micro-XRCE-DDS-Agent: udp4 -p ${XRCE_PORT}"
exec MicroXRCEAgent udp4 -p "${XRCE_PORT}"
