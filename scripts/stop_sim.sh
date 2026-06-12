#!/usr/bin/env bash
#
# stop_sim.sh — аккуратно гасит все процессы симуляции Фазы 2.
#
# Зачем: после Ctrl-C в `ros2 launch` сервер Gazebo (`gz sim -s`, его PX4 поднимает
# отдельно) часто остаётся «осиротевшим» и грузит CPU/GPU вхолостую. Этот скрипт
# добивает все хвосты одной командой. Идемпотентен — на пустой системе просто ничего
# не находит.
#
# НЕ трогает посторонние процессы: бьём только по характерным для нашего стека шаблонам.
#
set -uo pipefail   # без -e: pkill возвращает 1, когда процессов нет — это норма

# Шаблоны процессов нашего стека (pgrep/pkill -f по командной строке).
PATTERNS=(
  "px4_sitl_default/bin/px4"   # сам автопилот PX4 SITL
  "gz sim"                     # сервер/клиент Gazebo
  "MicroXRCEAgent"             # мост PX4 ↔ ROS2 (агент)
  "parameter_bridge"           # мост камеры Gazebo → ROS2
  "tail -n +1 -f /tmp/px4cmds" # вспомогательный канал команд (если использовался)
)

joined="$(IFS='|'; echo "${PATTERNS[*]}")"

# pgrep -f матчит и сам grep/скрипт по слову — исключаем себя по PID.
running() { pgrep -f "${joined}" 2>/dev/null | grep -vx "$$"; }

if [[ -z "$(running)" ]]; then
  echo "✓ Процессов симуляции не найдено — гасить нечего."
  exit 0
fi

echo "→ Найдены процессы симуляции, гашу (SIGTERM):"
pgrep -af "${joined}" | grep -viE "stop_sim|pgrep|grep "
for p in "${PATTERNS[@]}"; do pkill -f "$p" 2>/dev/null || true; done

# Даём 2 с на корректное завершение, затем добиваем уцелевших SIGKILL.
for _ in 1 2 3 4; do
  [[ -z "$(running)" ]] && break
  sleep 0.5
done
if [[ -n "$(running)" ]]; then
  echo "→ Кто-то не закрылся за 2 с, добиваю (SIGKILL)."
  for p in "${PATTERNS[@]}"; do pkill -9 -f "$p" 2>/dev/null || true; done
  sleep 0.5
fi

if [[ -z "$(running)" ]]; then
  echo "✓ Симуляция остановлена."
else
  echo "⚠ Остались процессы — проверь вручную:" >&2
  running >&2
  exit 1
fi
