#!/usr/bin/env bash
#
# install_system_deps.sh — системный (apt) слой стека: ROS2 Humble + Gazebo Harmonic
# + мосты (ros_gz, micro-XRCE-DDS-Agent). Ставится один раз в систему, НЕ в venv.
#
# Источник истины по версиям — docs/project_plan.md §4 / configs/simulator/stack.md.
#
# ⚠️ ВНИМАНИЕ:
#   - Скрипт использует sudo и правит apt-репозитории системы. Просмотри его перед запуском.
#   - Машина — Linux Mint 21.3 (база Jammy), но Mint отдаёт свой codename (virginia).
#     ROS2/Gazebo-репо собраны под Ubuntu-codename'ы, поэтому здесь жёстко форсим `jammy`.
#   - PX4-Autopilot и px4_msgs здесь НЕ ставятся — это сборка из исходников на Фазе 2.
#
set -euo pipefail

UBUNTU_CODENAME="jammy"          # форсим вместо Mint-codename (см. stack.md)
ARCH="$(dpkg --print-architecture)"
XRCE_AGENT_VERSION="v2.4.3"      # Micro-XRCE-DDS-Agent под PX4 v1.15 (подтвердить по docs.px4.io)
XRCE_SRC_DIR="${HOME}/src/Micro-XRCE-DDS-Agent"  # вне репозитория проекта

echo "=== Системный стек: ROS2 Humble + Gazebo Harmonic (codename=${UBUNTU_CODENAME}) ==="

# --- 0. Базовые утилиты + universe ---
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe

# --- 1. Репозиторий ROS2 (packages.ros.org) ---
if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
  echo "→ Добавляю apt-репозиторий ROS2"
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
else
  echo "→ Репозиторий ROS2 уже добавлен"
fi

# --- 2. Репозиторий Gazebo (packages.osrfoundation.org) — для gz-harmonic ---
if [[ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]]; then
  echo "→ Добавляю apt-репозиторий Gazebo (OSRF)"
  sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${UBUNTU_CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
else
  echo "→ Репозиторий Gazebo уже добавлен"
fi

# --- 3. Установка пакетов ---
# Примечание: micro-XRCE-DDS-Agent НЕТ в apt как ros-humble-* пакета. На Mint snap
# заблокирован, поэтому агента собираем из исходников ниже (шаг 5). cmake/build-essential
# нужны и для него, и для сборки colcon-пакетов на Фазе 1.
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools \
  gz-harmonic \
  ros-humble-ros-gzharmonic \
  cmake \
  build-essential \
  git

# --- 4. rosdep (нужен на Фазе 1 для разрешения зависимостей пакетов) ---
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  echo "→ Инициализирую rosdep"
  sudo rosdep init
fi
rosdep update

# --- 5. Micro-XRCE-DDS-Agent (мост PX4 ↔ ROS2) — сборка из исходников ---
# Нет apt-пакета; snap на Mint заблокирован. Собираем фиксированную версию из git.
# Идемпотентно: если бинарь уже в PATH — пропускаем.
if command -v MicroXRCEAgent > /dev/null 2>&1; then
  echo "→ MicroXRCEAgent уже установлен ($(command -v MicroXRCEAgent)) — пропускаю сборку"
else
  echo "→ Собираю Micro-XRCE-DDS-Agent ${XRCE_AGENT_VERSION} из исходников"
  if [[ ! -d "${XRCE_SRC_DIR}" ]]; then
    git clone -b "${XRCE_AGENT_VERSION}" \
      https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "${XRCE_SRC_DIR}"
  fi
  cmake -S "${XRCE_SRC_DIR}" -B "${XRCE_SRC_DIR}/build"
  cmake --build "${XRCE_SRC_DIR}/build" --parallel "$(nproc)"
  sudo cmake --install "${XRCE_SRC_DIR}/build"
  sudo ldconfig /usr/local/lib/
fi

cat <<EOF

✅ Системный стек установлен.

Проверка:
    source /opt/ros/humble/setup.bash
    ros2 --help               # ROS2 на месте
    gz sim --version          # Gazebo Harmonic (ждём gz-sim 8.x)
    MicroXRCEAgent --help     # мост PX4 ↔ ROS2 собран

Дальше — venv с pip-зависимостями:
    scripts/setup_venv.sh
EOF
