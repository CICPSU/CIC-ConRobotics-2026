#!/usr/bin/env bash

# ============================================================
# CIC Construction Robotics
# ROS 2 Zenoh network setup helper
#
# Usage:
#
#   Zenoh router:
#     source network/setup_zenoh.sh router
#     source network/setup_zenoh.sh router ros-pc
#     source network/setup_zenoh.sh router ros-backup-pc
#
#   ROS 2 client:
#     source network/setup_zenoh.sh client <device>
#     source network/setup_zenoh.sh client <device> <router-device>
#
# Examples:
#
#   Normal configuration - ROS PC is router:
#     source network/setup_zenoh.sh router ros-pc
#     source network/setup_zenoh.sh client ros-pc
#     source network/setup_zenoh.sh client dumptruck1
#     source network/setup_zenoh.sh client excavator3
#
#   Backup configuration - Backup ROS PC is router:
#     source network/setup_zenoh.sh router ros-backup-pc
#     source network/setup_zenoh.sh client ros-backup-pc ros-backup-pc
#     source network/setup_zenoh.sh client dumptruck1 ros-backup-pc
#     source network/setup_zenoh.sh client excavator3 ros-backup-pc
#
# Backward compatibility:
#
#   If no router device is specified, ros-pc is used.
#
#     source network/setup_zenoh.sh router
#     source network/setup_zenoh.sh client excavator3
#
#   are equivalent to:
#
#     source network/setup_zenoh.sh router ros-pc
#     source network/setup_zenoh.sh client excavator3 ros-pc
#
# This script:
#   1. Loads device addresses from devices.sh
#   2. Sets ROS_DOMAIN_ID
#   3. Sets RMW_IMPLEMENTATION=rmw_zenoh_cpp
#   4. Selects the Zenoh router
#   5. Configures the Zenoh router/client connection
#
# IMPORTANT:
#   This script must be sourced:
#
#     source network/setup_zenoh.sh ...
#
# ============================================================


# ------------------------------------------------------------
# Locate this script
# ------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"


# ------------------------------------------------------------
# Load device registry
# ------------------------------------------------------------

if [ ! -f "${SCRIPT_DIR}/devices.sh" ]; then
    echo "ERROR: Device registry not found:"
    echo "  ${SCRIPT_DIR}/devices.sh"
    return 1 2>/dev/null || exit 1
fi

source "${SCRIPT_DIR}/devices.sh"


# ------------------------------------------------------------
# Verify arguments
# ------------------------------------------------------------

if [ "$#" -lt 1 ]; then
    echo "ERROR: Zenoh role was not specified."
    echo
    echo "Usage:"
    echo "  source network/setup_zenoh.sh router [router-device]"
    echo "  source network/setup_zenoh.sh client <device> [router-device]"
    echo
    echo "Router devices:"
    echo "  ros-pc"
    echo "  ros-backup-pc"
    echo
    echo "Examples:"
    echo "  source network/setup_zenoh.sh router ros-pc"
    echo "  source network/setup_zenoh.sh router ros-backup-pc"
    echo "  source network/setup_zenoh.sh client excavator3 ros-pc"
    echo "  source network/setup_zenoh.sh client excavator3 ros-backup-pc"
    return 1 2>/dev/null || exit 1
fi


ROLE="$1"


# ------------------------------------------------------------
# Resolve device name to IP address
#
# User-facing names are intentionally simple.
#
# Existing underscore-style names are also accepted for
# backward compatibility.
# ------------------------------------------------------------

resolve_device_ip() {

    case "$1" in

        ros-pc|ros_pc)
            echo "${ROS_PC}"
            ;;

        ros-laptop|ros_laptop)
            echo "${ROS_Laptop}"
            ;;

        ros-backup-pc|ros_backup_pc|ros-laptop-backup|ros_laptop_backup)
            echo "${ROS_Laptop_Backup}"
            ;;

        dumptruck1|dumptruck_01)
            echo "${DUMPTRUCK_01}"
            ;;

        dumptruck2|dumptruck_02)
            echo "${DUMPTRUCK_02}"
            ;;

        dumptruck3|dumptruck_03)
            echo "${DUMPTRUCK_03}"
            ;;

        dumptruck4|dumptruck_04)
            echo "${DUMPTRUCK_04}"
            ;;

        dumptruck5|dumptruck_05)
            echo "${DUMPTRUCK_05}"
            ;;

        excavator1|excavator_01)
            echo "${EXCAVATOR_01}"
            ;;

        excavator2|excavator_02)
            echo "${EXCAVATOR_02}"
            ;;

        excavator3|excavator_03)
            echo "${EXCAVATOR_03}"
            ;;

        excavator4|excavator_04)
            echo "${EXCAVATOR_04}"
            ;;

        excavator5|excavator_05)
            echo "${EXCAVATOR_05}"
            ;;

        excavator6|excavator_06)
            echo "${EXCAVATOR_06}"
            ;;

        excavator7|excavator_07)
            echo "${EXCAVATOR_07}"
            ;;
        *)
            return 1
            ;;

    esac
}


# ------------------------------------------------------------
# Normalize router device name
# ------------------------------------------------------------

normalize_router_name() {

    case "$1" in

        ros-pc|ros_pc)
            echo "ros-pc"
            ;;

        ros-backup-pc|ros_backup_pc|ros-laptop-backup|ros_laptop_backup)
            echo "ros-backup-pc"
            ;;

        *)
            return 1
            ;;

    esac
}


# ------------------------------------------------------------
# Common ROS 2 / Zenoh settings
# ------------------------------------------------------------

export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_zenoh_cpp


# ------------------------------------------------------------
# Remove DDS-specific environment variables
# ------------------------------------------------------------

unset ROS_STATIC_PEERS
unset ROS_AUTOMATIC_DISCOVERY_RANGE
unset ROS_AUTOMATIC_DISCOVERY


# ------------------------------------------------------------
# Configure Zenoh role
# ------------------------------------------------------------

case "${ROLE}" in

    router)

        # ----------------------------------------------------
        # Arguments
        #
        # Default router:
        #   ros-pc
        #
        # Optional:
        #   ros-backup-pc
        # ----------------------------------------------------

        ROUTER_NAME="${2:-ros-pc}"

        NORMALIZED_ROUTER_NAME="$(normalize_router_name "${ROUTER_NAME}")"

        if [ $? -ne 0 ] || [ -z "${NORMALIZED_ROUTER_NAME}" ]; then
            echo "ERROR: Invalid Zenoh router device: ${ROUTER_NAME}"
            echo
            echo "Available router devices:"
            echo "  ros-pc"
            echo "  ros-backup-pc"
            return 1 2>/dev/null || exit 1
        fi

        ROUTER_NAME="${NORMALIZED_ROUTER_NAME}"

        ROUTER_IP="$(resolve_device_ip "${ROUTER_NAME}")"

        if [ $? -ne 0 ] || [ -z "${ROUTER_IP}" ]; then
            echo "ERROR: Could not resolve router IP: ${ROUTER_NAME}"
            return 1 2>/dev/null || exit 1
        fi


        # ----------------------------------------------------
        # Zenoh Router
        #
        # rmw_zenohd runs on the selected router machine.
        #
        # No ZENOH_CONFIG_OVERRIDE is required for rmw_zenohd.
        # ----------------------------------------------------

        unset ZENOH_CONFIG_OVERRIDE

        export CIC_ZENOH_ROLE="router"
        export CIC_ZENOH_DEVICE="${ROUTER_NAME}"
        export CIC_ZENOH_ROUTER="${ROUTER_NAME}"
        export CIC_ZENOH_ROUTER_IP="${ROUTER_IP}"


        echo
        echo "Zenoh router environment configured"
        echo "----------------------------------------"
        echo "Router device      : ${ROUTER_NAME}"
        echo "Router IP          : ${ROUTER_IP}"
        echo "ROS_DOMAIN_ID      : ${ROS_DOMAIN_ID}"
        echo "RMW_IMPLEMENTATION : ${RMW_IMPLEMENTATION}"
        echo "----------------------------------------"
        echo
        echo "Start the router with:"
        echo
        echo "  ros2 run rmw_zenoh_cpp rmw_zenohd"
        echo
        ;;


    client)

        # ----------------------------------------------------
        # Client arguments
        #
        # $2 = client device
        # $3 = router device (optional, default ros-pc)
        # ----------------------------------------------------

        DEVICE_NAME="${2:-}"
        ROUTER_NAME="${3:-ros-pc}"


        # ----------------------------------------------------
        # Client requires a device name
        # ----------------------------------------------------

        if [ -z "${DEVICE_NAME}" ]; then
            echo "ERROR: Client device was not specified."
            echo
            echo "Usage:"
            echo "  source network/setup_zenoh.sh client <device> [router-device]"
            echo
            echo "Examples:"
            echo "  source network/setup_zenoh.sh client ros-pc"
            echo "  source network/setup_zenoh.sh client dumptruck1"
            echo "  source network/setup_zenoh.sh client excavator3"
            echo "  source network/setup_zenoh.sh client excavator3 ros-backup-pc"
            return 1 2>/dev/null || exit 1
        fi


        # ----------------------------------------------------
        # Resolve client device
        # ----------------------------------------------------

        DEVICE_IP="$(resolve_device_ip "${DEVICE_NAME}")"

        if [ $? -ne 0 ] || [ -z "${DEVICE_IP}" ]; then
            echo "ERROR: Unknown device name: ${DEVICE_NAME}"
            echo
            echo "Available examples:"
            echo "  ros-pc"
            echo "  ros-laptop"
            echo "  ros-backup-pc"
            echo "  dumptruck1"
            echo "  dumptruck2"
            echo "  dumptruck3"
            echo "  dumptruck4"
            echo "  dumptruck5"
            echo "  excavator1"
            echo "  excavator2"
            echo "  excavator3"
            echo "  excavator4"
            echo "  excavator5"
            echo "  excavator6"
            echo "  excavator7"

            return 1 2>/dev/null || exit 1
        fi


        # ----------------------------------------------------
        # Resolve router device
        # ----------------------------------------------------

        NORMALIZED_ROUTER_NAME="$(normalize_router_name "${ROUTER_NAME}")"

        if [ $? -ne 0 ] || [ -z "${NORMALIZED_ROUTER_NAME}" ]; then
            echo "ERROR: Invalid Zenoh router device: ${ROUTER_NAME}"
            echo
            echo "Available router devices:"
            echo "  ros-pc"
            echo "  ros-backup-pc"
            return 1 2>/dev/null || exit 1
        fi

        ROUTER_NAME="${NORMALIZED_ROUTER_NAME}"

        ROUTER_IP="$(resolve_device_ip "${ROUTER_NAME}")"

        if [ $? -ne 0 ] || [ -z "${ROUTER_IP}" ]; then
            echo "ERROR: Could not resolve router IP: ${ROUTER_NAME}"
            return 1 2>/dev/null || exit 1
        fi


        # ----------------------------------------------------
        # Determine router endpoint
        #
        # If this client is running on the router machine,
        # connect through localhost.
        #
        # Otherwise connect through the router's network IP.
        # ----------------------------------------------------

        DEVICE_NORMALIZED="${DEVICE_NAME}"

        case "${DEVICE_NAME}" in

            ros_pc)
                DEVICE_NORMALIZED="ros-pc"
                ;;

            ros_backup_pc|ros-laptop-backup|ros_laptop_backup)
                DEVICE_NORMALIZED="ros-backup-pc"
                ;;

        esac


        if [ "${DEVICE_NORMALIZED}" = "${ROUTER_NAME}" ]; then
            ROUTER_ENDPOINT="tcp/127.0.0.1:7447"
        else
            ROUTER_ENDPOINT="tcp/${ROUTER_IP}:7447"
        fi


        # ----------------------------------------------------
        # Configure rmw_zenoh_cpp client
        # ----------------------------------------------------

        export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"${ROUTER_ENDPOINT}\"]"

        export CIC_ZENOH_ROLE="client"
        export CIC_ZENOH_DEVICE="${DEVICE_NAME}"
        export CIC_ZENOH_ROUTER="${ROUTER_NAME}"
        export CIC_ZENOH_ROUTER_IP="${ROUTER_IP}"


        echo
        echo "Zenoh client environment configured"
        echo "----------------------------------------"
        echo "Client device      : ${DEVICE_NAME}"
        echo "Client IP          : ${DEVICE_IP}"
        echo "Router device      : ${ROUTER_NAME}"
        echo "Router IP          : ${ROUTER_IP}"
        echo "Router endpoint    : ${ROUTER_ENDPOINT}"
        echo "ROS_DOMAIN_ID      : ${ROS_DOMAIN_ID}"
        echo "RMW_IMPLEMENTATION : ${RMW_IMPLEMENTATION}"
        echo "----------------------------------------"
        echo
        ;;


    *)

        echo "ERROR: Unknown Zenoh role: ${ROLE}"
        echo
        echo "Available roles:"
        echo "  router"
        echo "  client"
        return 1 2>/dev/null || exit 1
        ;;

esac