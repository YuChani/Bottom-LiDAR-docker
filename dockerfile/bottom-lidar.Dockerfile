# Base Image
FROM ubuntu:22.04

# 환경 변수 설정
ENV DEBIAN_FRONTEND=noninteractive

# 1. 필수 빌드 도구 및 라이브러리 설치
# (기존과 동일 + protobuf-compiler 추가: Foxglove 일부 기능이 쓸 수 있음)
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    libeigen3-dev \
    libpcl-dev \
    libboost-all-dev \
    libtbb-dev \
    nlohmann-json3-dev \
    libwebsocketpp-dev \
    libasio-dev \
    protobuf-compiler \
    libprotobuf-dev \
    && rm -rf /var/lib/apt/lists/*

# 작업용 임시 폴더
WORKDIR /tmp

# 2. GTSAM 소스 빌드 & 설치 (기존과 동일)
RUN git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && \
    git checkout 4.2a9 && \ 
    mkdir build && cd build && \
    cmake .. \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_STATIC_LIBRARY=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    && make -j$(nproc) && make install && \
    cd /tmp && rm -rf gtsam

# ------------------------------------------------------------------
# 3. [NEW] Foxglove WebSocket Protocol (C++) 설치
# 설명: C++ 코드에서 Foxglove Studio로 실시간 데이터를 쏘기 위한 라이브러리
# ------------------------------------------------------------------
RUN git clone https://github.com/foxglove/ws-protocol.git && \
    cd ws-protocol/cpp && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && make install && \
    cd /tmp && rm -rf ws-protocol

# ------------------------------------------------------------------
# 4. [NEW] MCAP (C++) 설치
# 설명: ROS bag 대신 .mcap 파일로 로그를 저장하기 위한 라이브러리 (필수 추천)
# ------------------------------------------------------------------
RUN git clone https://github.com/foxglove/cpp-mcap.git && \
    cd cpp-mcap && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DMCAP_BUILD_EXAMPLES=OFF && \
    make -j$(nproc) && make install && \
    cd /tmp && rm -rf cpp-mcap

# 5. 작업 디렉토리 설정
WORKDIR /root/workspace

# 6. 컨테이너 시작
CMD ["/bin/bash"]