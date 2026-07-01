#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")/../third_party/LiDAR_IMU_Init/docker"
docker build -f Dockerfile.noetic_local -t li_init:noetic-local .
