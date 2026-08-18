#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

bash "$SCRIPT_DIR/capture_image_goal.sh"
bash "$SCRIPT_DIR/run_imagegoal_evaluator.sh" capture
