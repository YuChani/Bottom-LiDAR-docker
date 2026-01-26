FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# 1. 필수 빌드 도구 및 기본 라이브러리 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget unzip pkg-config \
    gpg-agent software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# 2. PPA 추가 (GTSAM, Iridescence)
RUN add-apt-repository ppa:koide3/iridescence -y && \
    add-apt-repository ppa:koide3/gtsam -y && \
    apt-get update

# 3. 모든 필수 의존성 설치
RUN apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libboost-system-dev \
    libboost-timer-dev \
    libboost-thread-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-program-options-dev \
    libboost-serialization-dev \
    libboost-date-time-dev \
    libboost-iostreams-dev \
    libboost-regex-dev \
    libomp-dev \
    libtbb-dev \
    libmetis-dev \
    libgtest-dev \
    clang lld \
    ca-certificates \
    libglm-dev \
    libglfw3-dev \
    libpng-dev \
    libjpeg-dev \
    libspdlog-dev \
    libiridescence-dev \
    libgtsam-no-tbb-dev=4.3.0-1* \
    && rm -rf /var/lib/apt/lists/*

# 4. PCL 설치 (옵션)
ARG INSTALL_PCL=1
RUN if [ "$INSTALL_PCL" = "1" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
        libpcl-dev pcl-tools \
        && rm -rf /var/lib/apt/lists/*; \
    fi
WORKDIR /work
CMD ["bash"]
