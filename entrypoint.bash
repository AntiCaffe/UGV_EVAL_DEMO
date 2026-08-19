#!/usr/bin/env bash

set -euo pipefail

#이미지 이름
IMAGE="anticaffe/ugv_project:1.0.0"

#컨테이너 이름
CONTAINER_NAME="ugv_project"

#프로젝트 폴더 경로
PROJECT_DIR="/home/ivl/UGV_EVAL_DEMO" 

if (( $# == 0 )); then
  container_command=(bash)
else
  container_command=("$@")
fi

container_bootstrap='set -e
(cd /project/OpenPCDet && python setup.py develop)
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source /project/install/setup.bash
exec "$@"'

exec docker run --rm -it \
  --name "${CONTAINER_NAME}" \
  --gpus all \
  --network host \
  --workdir /project \
  -e ROS_DOMAIN_ID=200 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e CUDA_VISIBLE_DEVICES=0 \
  -e DISPLAY="${DISPLAY:-}" \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e QT_X11_NO_MITSHM=1 \
  -v "${PROJECT_DIR}:/project" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/dri:/dev/dri \
  --device=/dev/dri \
  "${IMAGE}" \
  bash -lc "${container_bootstrap}" bash "${container_command[@]}"
