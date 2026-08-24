#!/usr/bin/env bash

# ============================================================
# CIC Construction Robotics
# ROS 2 network setup helper
#
# Usage:
#   source network/setup_network.sh dumptruck_01 dumptruck_03
#
# Example:
#   source network/setup_network.sh ros_pc dumptruck_01 excavator_02
#
# This script:
#   1. Sets ROS_DOMAIN_ID
#   2. Sets ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
#   3. Builds ROS_STATIC_PEERS from device names
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "${SCRIPT_DIR}/devices.sh"

export ROS_DOMAIN_ID=10
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

if [ "$#" -eq 0 ]; then
    echo "ERROR: No peer devices were specified."
    echo
    echo "Usage:"
    echo "  source network/setup_network.sh <device1> <device2> ..."
    echo
    echo "Examples:"
    echo "  source network/setup_network.sh dumptruck_01"
    echo "  source network/setup_network.sh dumptruck_01 dumptruck_03"
    echo "  source network/setup_network.sh ros_pc dumptruck_01 excavator_02"
    return 1 2>/dev/null || exit 1
fi


resolve_device_ip() {
    case "$1" in

        ros_pc)
            echo "${ROS_PC}"
            ;;

        ros_laptop)
            echo "${ROS_Laptop}"
            ;;

        ros_laptop_backup)
            echo "${ROS_Laptop_Backup}"
            ;;

        dumptruck_01)
            echo "${DUMPTRUCK_01}"
            ;;

        dumptruck_02)
            echo "${DUMPTRUCK_02}"
            ;;

        dumptruck_03)
            echo "${DUMPTRUCK_03}"
            ;;

        dumptruck_04)
            echo "${DUMPTRUCK_04}"
            ;;

        dumptruck_05)
            echo "${DUMPTRUCK_05}"
            ;;

        excavator_01)
            echo "${EXCAVATOR_01}"
            ;;

        excavator_02)
            echo "${EXCAVATOR_02}"
            ;;

        excavator_03)
            echo "${EXCAVATOR_03}"
            ;;

        excavator_04)
            echo "${EXCAVATOR_04}"
            ;;

        *)
            return 1
            ;;
    esac
}


PEERS=""

for DEVICE_NAME in "$@"; do

    DEVICE_IP="$(resolve_device_ip "${DEVICE_NAME}")"

    if [ $? -ne 0 ] || [ -z "${DEVICE_IP}" ]; then
        echo "ERROR: Unknown device name: ${DEVICE_NAME}"
        echo
        echo "Check:"
        echo "  ${SCRIPT_DIR}/devices.sh"
        return 1 2>/dev/null || exit 1
    fi

    if [ -z "${PEERS}" ]; then
        PEERS="${DEVICE_IP}"
    else
        PEERS="${PEERS};${DEVICE_IP}"
    fi

done

export ROS_STATIC_PEERS="${PEERS}"

echo
echo "ROS 2 network configured"
echo "----------------------------------------"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE}"
echo "ROS_STATIC_PEERS=${ROS_STATIC_PEERS}"
echo "----------------------------------------"
echo