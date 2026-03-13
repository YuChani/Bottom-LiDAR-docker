#!/bin/bash
# graphmode_5run_experiment.sh
# full_connection / pure_chain(window=2) / sparse_chain(window=4) 를 각 5회 반복 실행
# 결과 로그: graphmode_5run_{mode}_run{N}.log
#
# NOTE: full_connection=true 는 GUI 에서만 설정 가능 (CLI 인자 없음).

CONTAINER="bottom-lidar"
BUILD_DIR="/root/workdir/build"
BINARY="./lidar_registration_benchmark"
LOG_BASE="/home/chani/personal/Bottom-LiDAR-docker/artifacts/logs/graphmode"
RUNS=5

mkdir -p "${LOG_BASE}"

echo "=== GraphMode 5-Run Experiment ==="
echo "Started at: $(date)"
echo "Modes: full_connection | pure_chain(w=2) | sparse_chain(w=4)"
echo ""

# --- full_connection (기존 로그 복사, 결정론적 실험이므로 동일 결과) ---
echo "### [1/3] full_connection (copy from existing log) ###"
SRC="${LOG_BASE}/graphmode_full_connection.log"
for R in $(seq 1 $RUNS); do
    DST="${LOG_BASE}/graphmode_5run_full_connection_run${R}.log"
    cp "${SRC}" "${DST}"
    echo "  -> ${DST}"
done

# --- pure_chain (window=2) ---
echo ""
echo "### [2/3] pure_chain (--window 2) ###"
for R in $(seq 1 $RUNS); do
    LOG="${LOG_BASE}/graphmode_5run_pure_chain_run${R}.log"
    echo "  Run ${R}/${RUNS} ..."
    docker exec "${CONTAINER}" bash -c \
        "cd ${BUILD_DIR} && ${BINARY} --headless --window 2 2>&1" \
        | tee "${LOG}" > /dev/null
    echo "  -> ${LOG}"
done

# --- sparse_chain (window=4) ---
echo ""
echo "### [3/3] sparse_chain (--window 4) ###"
for R in $(seq 1 $RUNS); do
    LOG="${LOG_BASE}/graphmode_5run_sparse_chain_run${R}.log"
    echo "  Run ${R}/${RUNS} ..."
    docker exec "${CONTAINER}" bash -c \
        "cd ${BUILD_DIR} && ${BINARY} --headless --window 4 2>&1" \
        | tee "${LOG}" > /dev/null
    echo "  -> ${LOG}"
done

echo ""
echo "=== All runs completed at: $(date) ==="
echo "Total log files: $(ls ${LOG_BASE}/graphmode_5run_*.log 2>/dev/null | wc -l)"
