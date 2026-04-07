#!/bin/bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

START_DELAY="${START_DELAY:-never}" \
EXPERIMENT_DURATION_SEC="${EXPERIMENT_DURATION_SEC:-70.0}" \
AUTO_START_USE_STATE_MACHINE_TRIGGER="${AUTO_START_USE_STATE_MACHINE_TRIGGER:-false}" \
AUTO_START_USE_LOCAL_MIN_TRIGGER="${AUTO_START_USE_LOCAL_MIN_TRIGGER:-false}" \
"$ROOT_DIR/scripts/run_px4_sih_docking_experiment.sh"
