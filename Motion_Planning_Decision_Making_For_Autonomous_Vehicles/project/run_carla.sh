#!/bin/bash

set -e

if [[ -n "${CARLA_ROOT:-}" ]]; then
  CARLA_SCRIPT="${CARLA_ROOT%/}/CarlaUE4.sh"
elif [[ -x "/opt/carla-simulator/CarlaUE4.sh" ]]; then
  CARLA_SCRIPT="/opt/carla-simulator/CarlaUE4.sh"
elif [[ -x "/mnt/c/Users/jayan/Downloads/CARLA_0.9.9.4/WindowsNoEditor/CarlaUE4.exe" ]]; then
  CARLA_EXE="/mnt/c/Users/jayan/Downloads/CARLA_0.9.9.4/WindowsNoEditor/CarlaUE4.exe"
elif [[ -x "/mnt/c/Users/jayan/Downloads/CARLA_0.9.13/WindowsNoEditor/CarlaUE4.exe" ]]; then
  CARLA_EXE="/mnt/c/Users/jayan/Downloads/CARLA_0.9.13/WindowsNoEditor/CarlaUE4.exe"
else
  echo "CARLA not found. Set CARLA_ROOT to your CARLA install directory." >&2
  echo "Example: export CARLA_ROOT=/opt/carla-simulator" >&2
  exit 1
fi

if [[ -n "${CARLA_EXE:-}" ]]; then
  # Launch Windows CARLA from WSL using a Windows path.
  # NOTE: do NOT pass -opengl on Windows; the packaged Windows build only ships
  # DirectX-cooked shaders (no GLSL_430 cache), so -opengl aborts on startup.
  CARLA_EXE_WIN="$(wslpath -w "${CARLA_EXE}")"
  cmd.exe /C start "" "${CARLA_EXE_WIN}"
else
  SDL_VIDEODRIVER=offscreen "${CARLA_SCRIPT}" -opengl&
fi
