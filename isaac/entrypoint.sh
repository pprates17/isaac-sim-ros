#!/bin/bash
set -e


source /isaac-sim/setup_python_env.sh


source /build_ws/jazzy/jazzy_ws/install/local_setup.bash
source /build_ws/jazzy/isaac_sim_ros_ws/install/local_setup.bash


export FASTRTPS_DEFAULT_PROFILES_FILE=/jazzy_ws/fastdds.xml
export ROS_DOMAIN_ID=0

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ISAACSIM_PATH}/exts/isaacsim.ros2.bridge/jazzy/lib"

export ISAACSIM_PATH="/isaac-sim"
export ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
export ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"

isaac_run() {
    # ---------------------
    # === RUN ISAAC SIM ===
    # ---------------------
    if [ $# -eq 0 ]; then
        # No args → Launch full Isaac Sim GUI
        echo "🧠 Launching Isaac Sim GUI..."
        "${ISAACSIM}"

    elif [[ "$1" == --* ]]; then
        # Arguments start with "--" → pass them to Isaac Sim executable
        echo "⚙️  Launching Isaac Sim with options: $*"
        "${ISAACSIM}" "$@"

    elif [ -f "$1" ]; then
        # First argument is a Python file → run with Isaac Sim's Python
        local SCRIPT_PATH="$1"
        shift
        echo "🚀 Running Python script with Isaac Sim: $SCRIPT_PATH"
        "${ISAACSIM_PYTHON}" "$SCRIPT_PATH" "$@"

    else
        # Unrecognized input
        echo "❌ Unknown argument or file not found: '$1'"
        echo "Usage:"
        echo "  isaac_run                 → launch GUI"
        echo "  isaac_run my_script.py    → run script with IsaacSim Python"
        echo "  isaac_run --headless ...  → launch IsaacSim with CLI flags"
        return 1
    fi
}

cd /PegasusSimulator/extensions
$ISAACSIM_PYTHON -m pip install --editable pegasus.simulator

#exec /isaac-sim/runheadless.sh "$@"
exec ${ISAACSIM_PYTHON} /PegasusSimulator/examples/12_px4_matrice.py --/app/livestream/enabled=true   --/app/livestream/mode=webrtc   --/app/livestream/port=49100   --/app/livestream/publicEndpointAddress=100.119.95.25
