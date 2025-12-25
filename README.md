# LIMO (Lidar-Monocular Visual Odometry)

This repository contains the LIMO project, restructured to be ROS-independent.

## Prerequisites

Ensure you have the following dependencies installed:

*   CMake (>= 3.20)
*   Ninja
*   Eigen3
*   OpenCV
*   Ceres Solver
*   PCL (Point Cloud Library)
*   Boost
*   libpng

## Build Instructions

1.  Create a build directory:
    ```bash
    mkdir build && cd build
    ```

2.  Configure the project using CMake and Ninja:
    ```bash
    cmake -GNinja ..
    ```

3.  Build the project:
    ```bash
    ninja
    ```

## Running

After building, the executable `limo_with_kitti` will be located in the `build/modules/apps` directory.

```bash
./modules/apps/limo_with_kitti
```
