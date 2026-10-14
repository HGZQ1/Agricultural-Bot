# Source this file in each terminal for the manual small-fruit harvest test.
AGRI_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$AGRI_PROJECT_ROOT" || return 1
source /opt/ros/jazzy/setup.bash || return 1
source "$AGRI_PROJECT_ROOT/ros2_ws/install/setup.bash" || return 1
source "$AGRI_PROJECT_ROOT/sim_ws/install/setup.bash" || return 1

export ROS_DOMAIN_ID=185
export GZ_PARTITION=agri_close_harvest_user_20261014
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=2
export AGRI_DETECTOR_DEVICE=0
export AGRI_SIM_STEP_SIZE=0.01
export AGRI_SIM_TIME_FACTOR=1.0

export AGRI_HARVEST_ASSETS="$AGRI_PROJECT_ROOT/artifacts/close_harvest_20261014/small_fruit_50_60_candidate"
export AGRI_NAV_PARAMS="$AGRI_PROJECT_ROOT/ros2_ws/src/agri_navigation/config/nav2_close_harvest_omni_v8.yaml"
export AGRI_ARM_PARAMS="$AGRI_PROJECT_ROOT/artifacts/close_harvest_20261014/harvest_arm_worker_lower_scan_090.yaml"
export AGRI_LOCALIZATION_MAP="$AGRI_PROJECT_ROOT/ros2_ws/src/agri_navigation/maps/tomato_field_default.yaml"
export AGRI_KEEPOUT_MAP="$AGRI_PROJECT_ROOT/ros2_ws/src/agri_navigation/maps/tomato_field_default_keepout.yaml"
