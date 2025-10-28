#!/bin/bash
set -e

# -------------------------------
# 1. Source Isaac Sim Python env
# -------------------------------
source /isaac-sim/setup_python_env.sh

# -------------------------------
# 2. Source ROS 2 workspace
# (replace paths with your actual build paths)
# -------------------------------
source /build_ws/jazzy/jazzy_ws/install/local_setup.bash
source /build_ws/jazzy/isaac_sim_ros_ws/install/local_setup.bash

# -------------------------------
# 3. Set ROS environment variables
# -------------------------------
export FASTRTPS_DEFAULT_PROFILES_FILE=/jazzy_ws/fastdds.xml
export ROS_DOMAIN_ID=0

# -------------------------------
# 4. Launch Isaac Sim headless
# -------------------------------
exec /isaac-sim/runheadless.sh "$@"
