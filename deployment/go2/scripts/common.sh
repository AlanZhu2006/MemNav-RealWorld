#!/usr/bin/env bash

NAVDP_GO2_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAVDP_GO2_DIR="$(cd "$NAVDP_GO2_SCRIPT_DIR/.." && pwd)"
NAVDP_ROOT="$(cd "$NAVDP_GO2_DIR/../.." && pwd)"
NAVDP_VENV="${NAVDP_VENV:-$NAVDP_ROOT/.venv-navdp}"
NAVDP_ROS_SETUP="${NAVDP_ROS_SETUP:-/opt/ros/humble/setup.bash}"
NAVDP_REALSENSE_SETUP="${NAVDP_REALSENSE_SETUP:-/home/nvidia/twork/realsense_ws/install/setup.bash}"
NAVDP_MESSAGE_FILTERS_SETUP="${NAVDP_MESSAGE_FILTERS_SETUP:-/home/nvidia/twork/message_filters_ws/install/local_setup.bash}"
NAVDP_CUSPARSELT_DIR="${NAVDP_CUSPARSELT_DIR:-$NAVDP_VENV/opt/cusparselt}"

navdp_source_file() {
  local setup_file="$1"
  if [[ -f "$setup_file" ]]; then
    local had_nounset=0
    case $- in
      *u*) had_nounset=1; set +u ;;
    esac
    source "$setup_file"
    if [[ "$had_nounset" == "1" ]]; then
      set -u
    fi
  fi
}

navdp_source_ros() {
  if [[ ! -f "$NAVDP_ROS_SETUP" ]]; then
    echo "ROS setup not found: $NAVDP_ROS_SETUP" >&2
    return 1
  fi
  navdp_source_file "$NAVDP_ROS_SETUP"
  navdp_source_file "$NAVDP_MESSAGE_FILTERS_SETUP"
  navdp_source_file "$NAVDP_REALSENSE_SETUP"
}

navdp_activate_venv() {
  if [[ ! -x "$NAVDP_VENV/bin/python" ]]; then
    echo "NavDP virtual environment is missing: $NAVDP_VENV" >&2
    echo "Run: $NAVDP_GO2_SCRIPT_DIR/setup_jetson.sh" >&2
    return 1
  fi
  navdp_source_file "$NAVDP_VENV/bin/activate"
  if [[ -d "$NAVDP_CUSPARSELT_DIR/lib" ]]; then
    export LD_LIBRARY_PATH="$NAVDP_CUSPARSELT_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
  export PYTHONPATH="$NAVDP_GO2_DIR${PYTHONPATH:+:$PYTHONPATH}"
  export MPLCONFIGDIR="${MPLCONFIGDIR:-$NAVDP_ROOT/.cache/matplotlib}"
  export CUDA_MODULE_LOADING="${CUDA_MODULE_LOADING:-LAZY}"
  export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
  export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
  mkdir -p "$MPLCONFIGDIR"
}
