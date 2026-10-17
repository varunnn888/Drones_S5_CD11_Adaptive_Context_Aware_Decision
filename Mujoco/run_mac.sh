#!/usr/bin/env bash
set -Eeuo pipefail

# Always run from this project directory, regardless of where the script is called.
cd "$(dirname "$0")"

# Use an existing project environment; create one on first launch if needed.
if [[ ! -d ".venv" ]]; then
  if command -v python3.12 >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python3.12)"
  elif command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python3)"
  else
    echo "ERROR: Python 3 was not found. Install Python 3.12 (recommended) and run this script again."
    exit 1
  fi
  echo "Creating the project's .venv using: $PYTHON_BIN"
  "$PYTHON_BIN" -m venv .venv
fi

# Activate this project's own environment so `python` always exists here.
# shellcheck disable=SC1091
source ".venv/bin/activate"

# Install missing/out-of-date requirements. Already-installed packages are kept.
echo "Checking project dependencies..."
python -m pip install -r requirements.txt

echo "Starting ACMFDS dashboard..."
exec python app.py
