#!/usr/bin/env bash
set -euo pipefail

exec powershell -ExecutionPolicy Bypass -File "$(dirname "$0")/bf.ps1" "$@"
