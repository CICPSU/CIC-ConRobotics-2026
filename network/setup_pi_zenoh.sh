#!/usr/bin/env bash

# ============================================================
# CIC-ConRobotics-2026 Raspberry Pi Setup
# Ubuntu 24.04 + ROS 2 Jazzy + Zenoh (rmw_zenoh_cpp)
#
# Intended for a NEW Raspberry Pi running Ubuntu 24.04.
#
# Installs:
#   - System updates and basic development tools
#   - ROS 2 Jazzy
#   - rmw_zenoh_cpp
#   - ROS development tools
#   - SSH
#   - pigpio
#   - I2C support
#   - Excavator calibration Python environment
#   - RPi.GPIO
#   - Adafruit Blinka
#   - Adafruit ADS1x15 library
#
# IMPORTANT:
#   - This script DOES NOT modify ~/.bashrc.
#   - ROS must be sourced explicitly in each terminal.
#   - RMW_IMPLEMENTATION must be set explicitly when needed.
#   - pigpiod is NOT configured to start automatically.
#   - Zenoh router startup/configuration is NOT handled here.
#   - Robot-specific repository/configuration is NOT handled here.
# ============================================================

set -euo pipefail


# ============================================================
# Configuration
# ============================================================

ROS_DISTRO="jazzy"
RMW_IMPL="rmw_zenoh_cpp"
EXCAVATOR_ENV="${HOME}/excavator_env"


log() {
    echo
    echo "============================================================"
    echo "$1"
    echo "============================================================"
}


# ============================================================
# 0. Check operating system
# ============================================================

log "Checking operating system"

if [[ ! -f /etc/os-release ]]; then
    echo "ERROR: /etc/os-release not found."
    exit 1
fi

. /etc/os-release

if [[ "${ID:-}" != "ubuntu" ]]; then
    echo "WARNING: This script is intended for Ubuntu."
    echo "Detected OS: ${ID:-unknown}"
fi

if [[ "${VERSION_ID:-}" != "24.04" ]]; then
    echo "WARNING: This script is intended for Ubuntu 24.04."
    echo "Detected version: ${VERSION_ID:-unknown}"
fi

echo "OS:           ${PRETTY_NAME:-unknown}"
echo "Architecture: $(dpkg --print-architecture)"
echo "Hostname:     $(hostname)"
echo "User:         ${USER}"


# ============================================================
# 1. Update system and install basic tools
# ============================================================

log "Updating system and installing basic tools"

sudo apt update

sudo DEBIAN_FRONTEND=noninteractive apt upgrade -y

sudo apt install -y \
    software-properties-common \
    curl \
    wget \
    unzip \
    git \
    build-essential \
    python3-pip \
    python3-venv \
    python3-dev \
    openssh-server \
    ca-certificates \
    gnupg \
    lsb-release \
    i2c-tools \
    libgpiod-dev


# ============================================================
# 2. Enable Ubuntu Universe repository
# ============================================================

log "Enabling Ubuntu Universe repository"

sudo add-apt-repository -y universe
sudo apt update


# ============================================================
# 3. Add official ROS 2 apt source
# ============================================================

log "Adding ROS 2 apt repository"

ROS_APT_SOURCE_VERSION="$(
    curl -fsSL \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' \
    | head -n 1 \
    | awk -F'"' '{print $4}'
)"

if [[ -z "${ROS_APT_SOURCE_VERSION}" ]]; then
    echo "ERROR: Could not determine ros-apt-source release version."
    exit 1
fi

UBUNTU_CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"

if [[ -z "${UBUNTU_CODENAME}" ]]; then
    echo "ERROR: Could not determine Ubuntu codename."
    exit 1
fi

ROS_APT_DEB="/tmp/ros2-apt-source.deb"

curl -fsSL \
    -o "${ROS_APT_DEB}" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb"

sudo dpkg -i "${ROS_APT_DEB}"

rm -f "${ROS_APT_DEB}"

sudo apt update


# ============================================================
# 4. Install ROS 2 Jazzy and Zenoh RMW
# ============================================================

log "Installing ROS 2 Jazzy and Zenoh RMW"

sudo apt install -y \
    ros-jazzy-ros-base \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-demo-nodes-py \
    ros-jazzy-rmw-zenoh-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool


# ============================================================
# 5. Initialize rosdep
# ============================================================

log "Initializing rosdep"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
else
    echo "rosdep is already initialized."
fi

rosdep update


# ============================================================
# 6. Enable SSH
# ============================================================

log "Enabling SSH"

sudo systemctl enable ssh
sudo systemctl restart ssh


# ============================================================
# 7. Enable I2C
# ============================================================

log "Enabling I2C"

BOOT_CONFIG="/boot/firmware/config.txt"

if [[ -f "${BOOT_CONFIG}" ]]; then

    if grep -Eq \
        '^[[:space:]]*dtparam=i2c_arm=on' \
        "${BOOT_CONFIG}"; then

        echo "I2C is already enabled in ${BOOT_CONFIG}."

    else

        echo "" \
            | sudo tee -a "${BOOT_CONFIG}" >/dev/null

        echo "# Enable I2C for CIC-ConRobotics" \
            | sudo tee -a "${BOOT_CONFIG}" >/dev/null

        echo "dtparam=i2c_arm=on" \
            | sudo tee -a "${BOOT_CONFIG}" >/dev/null

        echo "I2C enabled in ${BOOT_CONFIG}."

    fi

else

    echo "WARNING: ${BOOT_CONFIG} was not found."
    echo "I2C could not be enabled automatically."

fi


# Load I2C kernel module for the current session if available.
sudo modprobe i2c-dev || true


# ============================================================
# 8. Install pigpio from source
# ============================================================

log "Installing pigpio"

PIGPIO_DIR="${HOME}/pigpio-master"
PIGPIO_ZIP="${HOME}/master.zip"

rm -rf "${PIGPIO_DIR}"
rm -f "${PIGPIO_ZIP}"

wget -q \
    -O "${PIGPIO_ZIP}" \
    https://github.com/joan2937/pigpio/archive/master.zip

unzip -q \
    "${PIGPIO_ZIP}" \
    -d "${HOME}"

cd "${PIGPIO_DIR}"

make -j"$(nproc)"

sudo make install

rm -f "${PIGPIO_ZIP}"

sudo ldconfig


# ============================================================
# 9. Create excavator calibration Python environment
# ============================================================

log "Creating excavator calibration environment"

if [[ -d "${EXCAVATOR_ENV}" ]]; then

    echo "Existing virtual environment found:"
    echo "  ${EXCAVATOR_ENV}"
    echo
    echo "Reusing existing environment."

else

    echo "Creating virtual environment:"
    echo "  ${EXCAVATOR_ENV}"

    python3 -m venv "${EXCAVATOR_ENV}"

fi


# Activate the virtual environment only inside this script.
# This does NOT modify ~/.bashrc.

set +u

# shellcheck disable=SC1091
source "${EXCAVATOR_ENV}/bin/activate"

set -u


# Upgrade pip inside excavator_env.

python -m pip install --upgrade pip


# Install Raspberry Pi / Adafruit dependencies.
#
# These packages are intentionally installed inside excavator_env,
# NOT into the system Python environment.

python -m pip install \
    RPi.GPIO \
    Adafruit-Blinka \
    adafruit-circuitpython-ads1x15


deactivate


# ============================================================
# 10. Verify ROS 2 and Zenoh installation
# ============================================================

log "Verifying ROS 2 and Zenoh installation"

# Source ROS ONLY for this verification step.
# Nothing is written to ~/.bashrc.

set +u

# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

set -u


# Set Zenoh ONLY for this script process.
# This is NOT persistent.

export RMW_IMPLEMENTATION="${RMW_IMPL}"


echo "ROS_DISTRO:          ${ROS_DISTRO}"
echo "RMW_IMPLEMENTATION:  ${RMW_IMPLEMENTATION}"

echo
echo "Zenoh RMW package location:"

ros2 pkg prefix rmw_zenoh_cpp


# ============================================================
# 11. Verify SSH
# ============================================================

log "Verifying SSH"

echo "SSH enabled:"
systemctl is-enabled ssh

echo
echo "SSH active:"
systemctl is-active ssh


# ============================================================
# 12. Verify pigpio
# ============================================================

log "Verifying pigpio"

if command -v pigpiod >/dev/null 2>&1; then

    echo "pigpiod found:"
    command -v pigpiod

else

    echo "ERROR: pigpiod command was not found."
    exit 1

fi


# ============================================================
# 13. Verify I2C tools
# ============================================================

log "Verifying I2C"

if command -v i2cdetect >/dev/null 2>&1; then

    echo "i2cdetect found:"
    command -v i2cdetect

else

    echo "ERROR: i2cdetect was not found."
    exit 1

fi


echo

if [[ -e /dev/i2c-1 ]]; then

    echo "I2C device currently available:"
    ls -l /dev/i2c-1

else

    echo "NOTE: /dev/i2c-1 is not currently available."
    echo "A reboot may be required after enabling I2C."

fi


# ============================================================
# 14. Verify excavator calibration environment
# ============================================================

log "Verifying excavator calibration environment"

set +u

# shellcheck disable=SC1091
source "${EXCAVATOR_ENV}/bin/activate"

set -u


python - <<'PYTHON_VERIFY'

import RPi.GPIO as GPIO

print("RPi.GPIO import: OK")


import board
import busio

print("Adafruit Blinka import: OK")


from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_ads1x15.ads1x15 import Pin

print("ADS1x15 import: OK")

PYTHON_VERIFY


deactivate


# ============================================================
# 15. Setup complete
# ============================================================

log "Setup complete"


cat <<EOF_DONE

Raspberry Pi base setup completed successfully.

Installed/configured:

  - Ubuntu system updates
  - ROS 2 Jazzy ros-base
  - rmw_zenoh_cpp
  - colcon
  - rosdep
  - vcstool
  - SSH server
  - pigpio
  - I2C support
  - i2c-tools
  - Excavator calibration environment:
      ${EXCAVATOR_ENV}
  - RPi.GPIO
  - Adafruit Blinka
  - Adafruit CircuitPython ADS1x15


============================================================
IMPORTANT: ROS 2 + ZENOH
============================================================

This script DOES NOT modify ~/.bashrc.

Configure every ROS terminal explicitly:

    source /opt/ros/jazzy/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp


============================================================
EXCAVATOR POTENTIOMETER CALIBRATION
============================================================

Activate the excavator Python environment:

    source ~/excavator_env/bin/activate

Then run:

    python3 pot_test.py

When finished:

    deactivate


============================================================
EXCAVATOR MOTOR TEST
============================================================

Start pigpio manually:

    sudo pigpiod

Then run:

    python3 full_extest.py


============================================================
I2C CHECK
============================================================

After the ADS1115 is connected:

    i2cdetect -y 1


============================================================
REBOOT
============================================================

A reboot is recommended after the initial setup:

    sudo reboot


============================================================
NEXT STEP
============================================================

After reboot:

  1. Clone CIC-ConRobotics-2026.
  2. Install/build the robot-specific packages.
  3. Apply the configuration for the specific robot/Pi.


============================================================
ZENOH NOTE
============================================================

rmw_zenoh_cpp normally expects a Zenoh router to be available.

Zenoh router configuration/startup is intentionally NOT handled by
this generic Raspberry Pi setup script.

EOF_DONE