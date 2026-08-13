#!/usr/bin/env bash

set -euo pipefail

#이미지 이름
IMAGE="ugv_tracker:1.0.0"

#프로젝트 폴더 경로
PROJECT_DIR="/home/ivl/UGV_EVAL_DEMO" 

if (( $# == 0 )); then
  container_command=(bash)
else
  container_command=("$@")
fi

exec docker run --rm -it \
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
  "${container_command[@]}"
