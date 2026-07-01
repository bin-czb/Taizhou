#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
OUTPUT_ROOT="${1:-${HOME}/fast_livo2_validation_bags}"
SESSION_NAME="${2:-validation_$(date +%Y%m%d_%H%M%S)}"

exec "${SCRIPT_DIR}/record_fast_livo_site_bag.sh" "${OUTPUT_ROOT}" "${SESSION_NAME}"
