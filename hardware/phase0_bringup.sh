#!/bin/bash
# Hardware bring-up and configuration for Robot Conga (4-Robot TurtleBot 4 fleet)
# Usage: ./phase0_bringup.sh [ROBOT_NAMESPACE]
# Example:
#   ./phase0_bringup.sh tb4_0   # Leader
#   ./phase0_bringup.sh tb4_1   # Follower 1
#   ./phase0_bringup.sh tb4_2   # Follower 2
#   ./phase0_bringup.sh tb4_3   # Follower 3

# --- 1. Basic Network and DDS Configuration ---
# Common ROS_DOMAIN_ID for all 4 robots in the fleet
export ROS_DOMAIN_ID=42

# Ensure FastRTPS middleware is used
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Point to custom FastDDS XML profile if needed
# SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# export FASTRTPS_DEFAULT_PROFILES_FILE="${SCRIPT_DIR}/fastdds.xml"

# Discovery Server (if used over Wi-Fi):
# export ROS_DISCOVERY_SERVER="192.168.1.100:11811"

# --- 2. Namespace Configuration ---
# 4 robots: tb4_0 (Leader), tb4_1 (Follower 1), tb4_2 (Follower 2), tb4_3 (Follower 3)
export ROBOT_NAMESPACE="${1:-tb4_1}"
export ROS_NAMESPACE=$ROBOT_NAMESPACE

echo "=== Configured Robot Conga Environment ==="
echo "Robot Namespace: $ROBOT_NAMESPACE"
echo "ROS_DOMAIN_ID:   $ROS_DOMAIN_ID"
echo "RMW:             $RMW_IMPLEMENTATION"
echo "=========================================="

# --- 3. Time Synchronization ---
# Ensure Chrony NTP synchronization is active across all robots
# sudo systemctl restart chronyd
# chronyc tracking

# --- 4. Validation Commands ---
echo "Validation checks:"
echo "  1. View topics:   ros2 topic list | grep $ROBOT_NAMESPACE"
echo "  2. Check odom Hz: ros2 topic hz /$ROBOT_NAMESPACE/odom"
echo "  3. Teleop test:   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/$ROBOT_NAMESPACE/cmd_vel"

