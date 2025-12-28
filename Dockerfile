FROM ubuntu:24.04

# Install essential build tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    libeigen3-dev \
    libgtest-dev \
    libopencv-dev \
    libboost-all-dev \
    libpng-dev \
    libceres-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Set up working directory
WORKDIR /workspace/limo

# Copy the project (assume context is set to repo root)
COPY . /workspace/limo

# Default build command
CMD ["/bin/bash"]
