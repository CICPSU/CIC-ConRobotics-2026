#!/usr/bin/env bash

# ============================================================
# CIC Construction Robotics
# ROS 2 Zenoh network setup helper
#
# Usage:
#
#   ROS PC - Zenoh router:
#     source network/setup_zenoh.sh router
#
#   ROS PC - ROS 2 nodes:
#     source network/setup_zenoh.sh client ros-pc
#
#   Dump trucks:
#     source network/setup_zenoh.sh client dumptruck1
#     source network/setup_zenoh.sh client dumptruck2
#     source network/setup_zenoh.sh client dumptruck3
#     source network/setup_zenoh.sh client dumptruck4
#     source network/setup_zenoh.sh client dumptruck5
#
#   Excavators:
#     source network/setup_zenoh.sh client excavator1
#
# This script:
#   1. Loads device addresses from devices.sh
#   2. Sets ROS_DOMAIN_ID
#   3. Sets RMW_IMPLEMENTATION=rmw_zenoh_cpp
#   4. Configures the Zenoh router/client connection
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
    echo "  source network/setup_zenoh.sh router"
    echo "  source network/setup_zenoh.sh client <device>"
    echo
    echo "Examples:"
    echo "  source network/setup_zenoh.sh router"
    echo "  source network/setup_zenoh.sh client ros-pc"
    echo "  source network/setup_zenoh.sh client dumptruck1"
    echo "  source network/setup_zenoh.sh client excavator1"
    return 1 2>/dev/null || exit 1
fi


ROLE="$1"
DEVICE_NAME="${2:-}"


# ------------------------------------------------------------
# Resolve device name to IP address
#
# User-facing names are intentionally simple:
#
#   ros-pc
#   dumptruck1
#   dumptruck2
#   excavator1
#
# Existing underscore-style names are also accepted for
# backward compatibility with setup_network.sh.
# ------------------------------------------------------------

resolve_device_ip() {

    case "$1" in

        ros-pc|ros_pc)
            echo "${ROS_PC}"
            ;;

        ros-laptop|ros_laptop)
            echo "${ROS_Laptop}"
            ;;

        ros-laptop-backup|ros_laptop_backup)
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
#
# These may remain in the shell if setup_network.sh was sourced
# previously. They are not used by rmw_zenoh_cpp and should not
# remain active when switching network backends.
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
        # Zenoh Router
        #
        # The central router runs on ROS_PC.
        #
        # No ZENOH_CONFIG_OVERRIDE is required for rmw_zenohd.
        # ROS nodes running on this same machine should use:
        #
        #   source network/setup_zenoh.sh client ros-pc
        #
        # in their own terminals.
        # ----------------------------------------------------

        unset ZENOH_CONFIG_OVERRIDE

        export CIC_ZENOH_ROLE="router"
        export CIC_ZENOH_DEVICE="ros-pc"

        echo
        echo "Zenoh router environment configured"
        echo "----------------------------------------"
        echo "Device             : ros-pc"
        echo "Router IP          : ${ROS_PC}"
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
        # Client requires a device name
        # ----------------------------------------------------

        if [ -z "${DEVICE_NAME}" ]; then
            echo "ERROR: Client device was not specified."
            echo
            echo "Usage:"
            echo "  source network/setup_zenoh.sh client <device>"
            echo
            echo "Examples:"
            echo "  source network/setup_zenoh.sh client ros-pc"
            echo "  source network/setup_zenoh.sh client dumptruck1"
            echo "  source network/setup_zenoh.sh client excavator1"
            return 1 2>/dev/null || exit 1
        fi


        DEVICE_IP="$(resolve_device_ip "${DEVICE_NAME}")"

        if [ $? -ne 0 ] || [ -z "${DEVICE_IP}" ]; then
            echo "ERROR: Unknown device name: ${DEVICE_NAME}"
            echo
            echo "Available examples:"
            echo "  ros-pc"
            echo "  ros-laptop"
            echo "  dumptruck1"
            echo "  dumptruck2"
            echo "  dumptruck3"
            echo "  dumptruck4"
            echo "  dumptruck5"
            echo "  excavator1"
            echo "  excavator2"
            echo "  excavator3"
            echo "  excavator4"
            return 1 2>/dev/null || exit 1
        fi


        # ----------------------------------------------------
        # ROS PC connects to local router.
        #
        # All remote machines connect to ROS_PC.
        # ----------------------------------------------------

        case "${DEVICE_NAME}" in

            ros-pc|ros_pc)

                ROUTER_ENDPOINT="tcp/127.0.0.1:7447"

                ;;

            *)

                ROUTER_ENDPOINT="tcp/${ROS_PC}:7447"

                ;;

        esac


        export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"${ROUTER_ENDPOINT}\"]"

        export CIC_ZENOH_ROLE="client"
        export CIC_ZENOH_DEVICE="${DEVICE_NAME}"


        echo
        echo "Zenoh client environment configured"
        echo "----------------------------------------"
        echo "Device             : ${DEVICE_NAME}"
        echo "Device IP          : ${DEVICE_IP}"
        echo "Router IP          : ${ROS_PC}"
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