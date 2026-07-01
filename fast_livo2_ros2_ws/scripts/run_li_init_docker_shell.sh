#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

docker run --rm -it \
  --net=host \
  --ipc=host \
  --shm-size=1gb \
  --volume="${WORKSPACE_ROOT}/third_party:/home/catkin_ws/src:rw" \
  --volume="${WORKSPACE_ROOT}/converted_bags_ros1:/bags:rw" \
  --name=li_init_noetic \
  li_init:noetic-local \
  /bin/bash
