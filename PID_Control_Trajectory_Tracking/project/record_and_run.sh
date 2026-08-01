#!/bin/bash
# Gives you time to focus the pygame/CARLA window and start recording
# before the controller connects. Usage (CARLA already running):
#   ./record_and_run.sh

set -e
cd "$(dirname "$0")"

COUNTDOWN="${1:-12}"

echo "=============================================="
echo "  RECORDING COUNTDOWN"
echo "=============================================="
echo "1. Make sure CARLA is already open and loaded."
echo "2. Get ready to focus the pygame window when it appears."
echo "3. Start recording with Win+Alt+R (Game Bar)."
echo "   Or start Win+G recording now on an empty area —"
echo "   you can switch focus to pygame when it opens."
echo ""
echo "Controller starts in ${COUNTDOWN} seconds..."
echo "=============================================="

for ((i=COUNTDOWN; i>0; i--)); do
  printf "\r  %2d  " "$i"
  sleep 1
done
echo ""
echo "Starting ./run_main_pid.sh now — leave the recorder running."
echo ""

exec ./run_main_pid.sh
