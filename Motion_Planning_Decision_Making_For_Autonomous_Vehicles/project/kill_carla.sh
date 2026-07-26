#!/bin/bash

set -euo pipefail

killed_any=0

kill_if_running() {
  local cmd="$1"
  if eval "$cmd" >/dev/null 2>&1; then
    killed_any=1
  fi
}

# Windows CARLA processes (from WSL)
if command -v taskkill.exe >/dev/null 2>&1; then
  kill_if_running "taskkill.exe /F /IM CarlaUE4.exe"
  kill_if_running "taskkill.exe /F /IM CarlaUE4-Win64-Shipping.exe"
fi

# Linux CARLA process names
kill_if_running "pkill -f CarlaUE4.sh"
kill_if_running "pkill -f CarlaUE4-Linux-Shipping"

if [[ "${killed_any}" -eq 1 ]]; then
  echo "Stopped CARLA process(es)."
else
  echo "No CARLA process found."
fi
