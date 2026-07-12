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
  --output <dir>            output directory for pose files (default: ./output)
  --rerun <mode>            spawn | connect[:<url>] | save:<path> | off (default: spawn)
  --max-frames <n>          stop after n frames (default: all)
  --semantic <dir>          directory with semantically segmented images (optional)
  --shrubbery-weight <w>    weight for vegetation landmarks with --semantic (default: 0.5)
  --mono                    camera-only mode, no lidar (requires --semantic)
  --camera-height <m>       camera height over ground for --mono (default: 1.65)
```

## Semantic segmentation (optional)

The pipeline runs fine on plain images. With semantically segmented images it additionally

- **downweights vegetation** landmarks (`--shrubbery-weight`) and treats sky and dynamic objects
  (vehicles, pedestrians, ...) as outliers,
- estimates the **ground plane from the labeled road pixels** instead of RANSAC and activates the
  ground-plane residuals and regularization in the bundle adjustment (landmarks on the road are
  pulled onto a per-keyframe plane, which stabilizes pitch and height),
- enables **`--mono`**: camera-only odometry without any lidar, where the metric scale comes from
  the camera height over the detected ground plane (`--camera-height`).

Label images are 8-bit PNGs with [cityscapes label ids](https://github.com/mcordts/cityscapesScripts)
(road=7, vegetation=21, sky=23, car=26, ...), one per frame (`000000.png`, ...).

Generate them with the bundled segmentation service (SegFormer-B0 fine-tuned on Cityscapes,
CPU inference, model baked into the image):

```bash
docker build -t limo-semantic semantic/
docker run --rm \
    -v /path/to/sequences/04/image_2:/in:ro \
    -v /path/to/sequences/04/semantic_2:/out \
    limo-semantic /in /out
```

Then:

```bash
# lidar + semantics
docker run --rm -v /path/to/sequences/04:/data:ro -v $PWD/out:/out limo \
    /data --output /out --rerun save:/out/limo.rrd --semantic /data/semantic_2

# camera only (no velodyne data needed at all)
docker run --rm -v /path/to/sequences/04:/data:ro -v $PWD/out:/out limo \
    /data --output /out --rerun off --mono --semantic /data/semantic_2
```

Measured on KITTI odometry sequence 04 (394 m; final position error, single runs — the solver has
a wall-clock budget, so expect ±1-2 % variance between runs):

| Configuration | Final error |
|---|---|
| lidar, plain images | 1-2 % |
| lidar + semantics, `--semantic-gp off` (label weights/outliers only) | ~3 % |
| lidar + semantics (full ground-plane machinery) | ~7 % |
| `--mono` + semantics (experimental) | scale under-estimated |

The label weighting is roughly at parity with the plain pipeline; the per-keyframe ground-plane
estimation currently costs accuracy on this sequence because the plane distances are free
parameters chained only by a neighbor regularization and drift in pitch on long runs — tuning
that (and the saturating robust losses that keep `--mono` near the scale of its motion prior) is
the next research step; the original LIMO gains also relied on a segmentation network fine-tuned
for the KITTI domain rather than an off-the-shelf Cityscapes model.

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
