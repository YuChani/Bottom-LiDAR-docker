#!/bin/bash
# window_sweep_experiment.sh
# sparse_connection_window 파라미터를 2~8로 순회하며 benchmark 실행
# 결과 로그: window_sweep_w{N}.log

set -e

CONTAINER="bottom-lidar"
BUILD_DIR="/root/workdir/build"
BINARY="./lidar_registration_benchmark"
LOG_DIR="/root/workdir"

echo "=== Window Sweep Experiment (window=2~8) ==="
echo "Started at: $(date)"

for W in 2 3 4 5 6 7 8; do
    LOG_FILE="${LOG_DIR}/window_sweep_w${W}.log"
    echo ""
    echo "--- Running window=${W} ---"
    docker exec "${CONTAINER}" bash -c \
        "cd ${BUILD_DIR} && ${BINARY} --headless --window ${W} 2>&1" \
        | tee "/home/chani/personal/Bottom-LiDAR-docker/window_sweep_w${W}.log"
    echo "--- Done window=${W} → window_sweep_w${W}.log ---"
done

echo ""
echo "=== All runs completed at: $(date) ==="
