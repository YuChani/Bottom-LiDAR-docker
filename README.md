# Bottom-LiDAR-docker

## gtsam_points (non-ROS) Docker (CPU)

This repo also provides a non-ROS, CPU-only Docker environment for projects that use
`koide3/gtsam_points` scan-matching factors (ICP/GICP/VGICP) to estimate relative pose (R,t).

### Build

```bash
docker build -f dockerfile/gtsam-points-cpu.Dockerfile -t gtsam-points-cpu:latest .
```

Optional (heavy): include PCL

```bash
docker build -f dockerfile/gtsam-points-cpu.Dockerfile --build-arg INSTALL_PCL=1 -t gtsam-points-cpu:latest .
```

### Run

```bash
docker run --rm -it -v "$PWD":/work -w /work gtsam-points-cpu:latest bash
```

### docker compose

Build:

```bash
docker compose -f docker-compose/gtsam-points-cpu_compose.yml build
```

Run:

```bash
docker compose -f docker-compose/gtsam-points-cpu_compose.yml run --rm gtsam-points-cpu
```

### Verify installation inside container

```bash
dpkg -l | egrep 'libgtsam|libgtsam-points' | cat
```

Sanity build (GTSAM + gtsam_points are linkable):

```bash
docker run --rm gtsam-points-cpu:latest bash -lc '
set -euo pipefail
mkdir -p /tmp/gtsam-points-test
cat > /tmp/gtsam-points-test/CMakeLists.txt <<"EOF"
cmake_minimum_required(VERSION 3.16)
project(gtsam_points_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gtsam_points REQUIRED)

add_executable(gtsam_points_test main.cpp)
target_link_libraries(gtsam_points_test PRIVATE gtsam_points::gtsam_points)
EOF

cat > /tmp/gtsam-points-test/main.cpp <<"EOF"
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>

int main() { return 0; }
EOF

cmake -S /tmp/gtsam-points-test -B /tmp/gtsam-points-test/build -G Ninja
cmake --build /tmp/gtsam-points-test/build -v
'
```

Note: With the koide3 PPA packages on Ubuntu 22.04, the exported GTSAM CMake target name is
`gtsam` (not `GTSAM::gtsam`). If your downstream CMake tries to link `GTSAM::gtsam`, update it
to use the PPA-provided target(s) instead.
