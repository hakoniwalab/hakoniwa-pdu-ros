#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
IMAGE_NAME="${HAKONIWA_PDU_ROS_NATIVE_IMAGE:-hakoniwa-pdu-ros-native-test}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-73}"

printf 'Building %s\n' "${IMAGE_NAME}"
docker build \
  --file "${SCRIPT_DIR}/Dockerfile" \
  --tag "${IMAGE_NAME}" \
  "${REPO_ROOT}"

printf 'Running native tests with ROS_DOMAIN_ID=%s and RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n' \
  "${ROS_DOMAIN_ID_VALUE}"
docker run --rm --init \
  --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}" \
  --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  "${IMAGE_NAME}"
