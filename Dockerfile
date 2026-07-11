# Build stage: compile LIMO and run the full test suite (the image only builds if tests pass).
FROM ubuntu:24.04 AS build

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    ca-certificates \
    libeigen3-dev \
    libgtest-dev \
    libopencv-dev \
    libboost-all-dev \
    libpng-dev \
    libceres-dev \
    libpcl-dev \
    libxcb1-dev \
    libxkbcommon-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/limo
COPY . /workspace/limo

RUN cmake -GNinja -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build \
    && ctest --test-dir build --output-on-failure

# Runtime stage: only the binary and its shared libraries.
FROM ubuntu:24.04

RUN apt-get update && apt-get install -y \
    libopencv-core406t64 \
    libopencv-imgproc406t64 \
    libopencv-imgcodecs406t64 \
    libpcl-filters1.14 \
    libpcl-sample-consensus1.14 \
    libpcl-search1.14 \
    libceres4t64 \
    libgomp1 \
    libpng16-16t64 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build /workspace/limo/build/modules/apps/limo_with_kitti /usr/local/bin/limo_with_kitti

ENTRYPOINT ["limo_with_kitti"]
CMD ["--help"]
