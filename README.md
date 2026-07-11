# LIMO — Lidar-Monocular Visual Odometry (ROS-free)

LIMO estimates the trajectory of a vehicle from a **single camera** and a **lidar**: features are
tracked in the images, their depth is estimated from the lidar point cloud, and a keyframe bundle
adjustment fuses both — the lidar depth anchors the metric scale that a mono pipeline alone cannot
observe ([paper](https://arxiv.org/abs/1807.07524)).

This repository is the ROS-independent version: one repository, one Dockerfile, one binary
(`limo_with_kitti`) that reads a KITTI odometry sequence, runs the full pipeline, writes poses for
evaluation and streams the trajectory, images, lidar clouds and reconstructed landmarks to
[rerun](https://rerun.io).

## Quick start (Docker)

```bash
git clone <this repo> limo && cd limo
docker build -t limo .          # compiles everything and runs the complete test suite
```

Run on a KITTI odometry sequence (directory with `image_2/`, `velodyne/`, `calib.txt`,
`times.txt`):

```bash
mkdir -p out
docker run --rm \
    -v /path/to/kitti/dataset/sequences/04:/data:ro \
    -v $PWD/out:/out \
    limo /data --output /out --rerun save:/out/limo.rrd
```

Afterwards `out/` contains:

| File | Content |
|---|---|
| `poses.txt` | one pose per frame, KITTI odometry format (3x4 row-major), ready for `evo` / KITTI devkit |
| `poses_keyframes.txt` | the subset of optimized keyframe poses |
| `limo.rrd` | rerun recording — open with `rerun out/limo.rrd` |

Live visualization instead of a recording: start `rerun` on the host and let the container
connect to it:

```bash
rerun &   # pip install rerun-sdk, or download the viewer from rerun.io
docker run --rm --net=host -v /path/to/sequences/04:/data:ro -v $PWD/out:/out \
    limo /data --output /out --rerun connect
```

### CLI

```
limo_with_kitti <KITTI sequence dir> [options]
  --output <dir>       output directory for pose files (default: ./output)
  --rerun <mode>       spawn | connect[:<url>] | save:<path> | off (default: spawn)
  --max-frames <n>     stop after n frames (default: all)
```

## Evaluation

The poses are those of the used camera (image_2), relative to the first frame. On KITTI odometry
sequence 04 (394 m) this pipeline reaches a final position error of ~1% of the path length with
metric scale purely from the lidar depth (no semantics yet, untuned parameters). With
[evo](https://github.com/MichaelGrupp/evo):

```bash
evo_traj kitti out/poses.txt --ref /path/to/kitti/dataset/poses/04.txt -p --plot_mode xz
evo_ape  kitti /path/to/kitti/dataset/poses/04.txt out/poses.txt -a
```

(Ground-truth poses refer to camera 0; camera 2 sits a constant ~6 cm beside it, which is
irrelevant for relative errors and absorbed by alignment for absolute ones.)

## Architecture

```
KITTI frame (color image + velodyne cloud)
  │  kitti_data_reader        C++23 coroutine generator; calib incl. P2 baseline offset
  ▼
grayscale + gamma correction  image_preproc
  ▼
feature tracking              feature_tracking_core (viso2 matcher)
  ▼
lidar depth per feature       mono_lidar_depth::TrackletDepthEstimator
  │                           (pixel-rect neighbors -> depth histogram blob -> local plane ->
  │                            ray intersection; RANSAC ground plane fallback for road points)
  ▼
keyframe selection            keyframe_bundle_adjustment (flow rejection)
  ▼
bundle adjustment             keyframe_bundle_adjustment (Ceres 2.2, manifolds; landmark
  │                           selection: cheirality, voxel sparsification, depth-guaranteed)
  ▼
poses.txt + rerun logging
```

Modules (all plain CMake targets, no ROS):

| Module | Purpose |
|---|---|
| `modules/matches_msg_types` | header-only tracklet/feature types shared across the pipeline |
| `modules/viso2` | libviso2 feature matcher |
| `modules/feature_tracking_core` | feature tracker on top of viso2 |
| `modules/image_preproc` | grayscale preprocessing (gamma/brightness correction) |
| `modules/mono_lidar_depth` | depth of monocular features from lidar (C++23, gtest suite) |
| `modules/robust_optimization` | robust Ceres solving with outlier trimming |
| `modules/keyframe_bundle_adjustment` | keyframe/landmark selection + bundle adjustment |
| `modules/apps` | KITTI reader, rerun visualization, the `limo_with_kitti` main |

### Semantic segmentation

The original LIMO weighted landmarks by semantic class (vegetation, ground, dynamic objects).
This port keeps all of that machinery wired — `Tracklet::label`, per-class weights in the bundle
adjustment, and a `SemanticPlane` ground-plane implementation — but currently runs with **uniform
weights and no label input**. When segmented images become available, plug them in at the
labeling seam in `limo_with_kitti.cpp` (`convertTracklets`) and pass them to
`TrackletDepthEstimator::process`.

## Building natively (without Docker)

Dependencies: CMake ≥ 3.28, a C++23 compiler (gcc 13+), Eigen3, OpenCV, PCL, Ceres ≥ 2.2, Boost,
libpng, gtest (see the Dockerfile for the exact apt packages).

```bash
cmake -GNinja -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
./build/modules/apps/limo_with_kitti /path/to/sequences/04 --rerun spawn
```

## Tests

`ctest` runs five suites: unit tests for `robust_optimization`,
`keyframe_bundle_adjustment`, `mono_lidar_depth` (40 tests incl. synthetic-scene integration
tests) and the KITTI reader, plus `test_limo_e2e`, which generates a synthetic KITTI sequence
with known motion, runs the actual `limo_with_kitti` binary on it and asserts the recovered
trajectory (including metric scale) against ground truth. The Docker build fails if any test
fails.

## License

See [LICENSE](modules/mono_lidar_depth/LICENSE) and the original
[LIMO repository](https://github.com/johannes-graeter/limo).
