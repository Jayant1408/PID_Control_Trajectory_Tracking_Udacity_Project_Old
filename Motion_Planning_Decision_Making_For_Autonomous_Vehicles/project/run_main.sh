#!/bin/bash

# ./starter_files/spiral_planner&
./Planning/build/spiral_planner&
sleep 1.0

# Prefer Windows CARLA PythonAPI if available (WSL + Windows CARLA).
WIN_CARLA_EGG="/mnt/c/Users/jayan/Downloads/CARLA_0.9.13/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.13-py3.7-win-amd64.egg"
if [[ -f "${WIN_CARLA_EGG}" ]]; then
  WIN_EGG_WIN="$(wslpath -w "${WIN_CARLA_EGG}")"
  # Windows python can't open UNC WSL paths, so copy script to Windows temp.
  WIN_TMP_DIR="/mnt/c/Users/jayan/Downloads/carla_tmp"
  mkdir -p "${WIN_TMP_DIR}"
  cp -f "$(pwd)/simulatorAPI.py" "${WIN_TMP_DIR}/simulatorAPI.py"
  SCRIPT_WIN="$(wslpath -w "${WIN_TMP_DIR}/simulatorAPI.py")"
  WIN_PYTHON_DEFAULT="/mnt/c/Users/jayan/AppData/Local/Programs/Python/Python37/python.exe"
  WIN_PYTHON_DEFAULT_WIN="$(wslpath -w "${WIN_PYTHON_DEFAULT}")"
  # Prefer Windows py launcher if available (Python 3.7 needed for this egg).
  if cmd.exe /C "py -3.7 -V" > /dev/null 2>&1; then
    cmd.exe /C "cd /d C:\\ && set PYTHONPATH=${WIN_EGG_WIN}&& py -3.7 ${SCRIPT_WIN}"
    exit $?
  fi

  if [[ -x "${WIN_PYTHON_DEFAULT}" ]]; then
    cmd.exe /C "cd /d C:\\ && set PYTHONPATH=${WIN_EGG_WIN}&& ${WIN_PYTHON_DEFAULT_WIN} ${SCRIPT_WIN}"
    exit $?
  fi

  WIN_PYTHON_CANDIDATES="$(cmd.exe /C where python 2>nul | tr -d '\r')"
  WIN_PYTHON=""
  while IFS= read -r candidate; do
    if [[ -n "${candidate}" && "${candidate}" != *"WindowsApps"* ]]; then
      WIN_PYTHON="${candidate}"
      break
    fi
  done <<< "${WIN_PYTHON_CANDIDATES}"
  if [[ -n "${WIN_PYTHON}" ]]; then
    cmd.exe /C "cd /d C:\\ && set PYTHONPATH=${WIN_EGG_WIN}&& ${WIN_PYTHON} ${SCRIPT_WIN}"
    exit $?
  fi

  echo "Windows Python 3.7 not found. Install it so the CARLA PythonAPI egg can load." >&2
  echo "Suggested: install Python 3.7 and ensure the 'py' launcher is available." >&2
  exit 1
fi

python3 simulatorAPI.py
