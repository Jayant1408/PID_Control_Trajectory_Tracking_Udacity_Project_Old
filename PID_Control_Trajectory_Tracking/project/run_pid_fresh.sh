#!/bin/bash

set -euo pipefail

echo "[1/5] Stopping old PID/controller processes..."
pkill -f "pid_controller|simulatorAPI.py|run_main_pid.sh" || true

echo "[2/5] Stopping old CARLA processes..."
taskkill.exe /F /IM CarlaUE4.exe /IM CarlaUE4-Win64-Shipping.exe >/dev/null 2>&1 || true
taskkill.exe /F /IM python.exe /IM py.exe >/dev/null 2>&1 || true

echo "[3/5] Launching CARLA..."
CARLA_SCRIPT="${HOME}/udacity_carla_local/Motion_Planning_Decision_Making_For_Autonomous_Vehicles/project/run_carla.sh"
"${CARLA_SCRIPT}"

echo "[4/5] Waiting for CARLA to initialize..."
sleep "${CARLA_WAIT:-30}"

echo "[5/5] Starting PID run..."
exec ./run_main_pid.sh
