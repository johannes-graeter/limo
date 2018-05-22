# Keyframe bundle adjustment

* This is a library for doing bundle adjustment with visual sensors
* It is supposed to be an add-on module to do temporal inference of the optimization graph in order to smooth the result
* In order to do that online a windowed approach is used
* Keyframes are instances in time which are used for the bundle adjustment, one keyframe may have several cameras (and therefore images) associated with it
* The selection of Keyframes tries to reduce the amount of redundant information while extending the time span covered by the optimization window to reduce drift
* Methodologies for Keyframe selection:
  * Difference in time
  * Difference in motion

* We use this library for combining Lidar with monocular vision in limo (Evaluated on Kitti benchmark.)

## Note

This is work in progress, I would appreciate feedback and experiences with this library.

### Try it out

If you just want to give it a quick peek, I prepared a ready-to-use virtualbox image (packed with Ubuntu 16.04.04, ros kinetic, ceres, mrt_cmake_modules and limo).

* Download it from [https://www.mrt.kit.edu/graeterweb/limo_core.ova](https://www.mrt.kit.edu/graeterweb/limo_core.ova).
* Find the library in ~/workspaces/limo/src/limo.
* Check out the unittests for examples on simulated data.
* Password for the vm-image is "1234".

## Usage

Here you find some snippets for principal understanding of the user interface. For running examples with simulated data, have a look at the unittests in limo/test/keyframe_bundle_adjustment.cpp; especially TEST(KeyFrameBundleAdjustment, solve_depth) and TEST(KeyFrameBundleAdjustment, solve) could be of interest.

* You need to add Keyframes to bundle_adjuster to add data

```cpp
// Beforehand you calculated:
// The tracklets with or wihout depth data keyframe_bundle_adjustment::Tracklets tracklets;
// The frame to frame motion estimation std::map<KeyframeId, Eigen::Isometry3d> pose_prior;

using namespace keyframe_bundle_adjustment;
// This is the main class where bundle adjustment is done.
BundleAdjusterKeyframes b;

// Create a pinhole camera, with intrinsics and extrinsics.
double focal_length=600;
Eigen::Vector2d principal_point(200.,100.);
Eigen::Isometry3d transform_cam_vehicle = Eigen::Isometry3d::Identity();
Camera::Ptr camera = std::make_shared<Camera>(focal_length, principal_point, transform_cam_vehicle;

// Create Keyframes from data.
// Timestamps here are only ints, but you should use the system time of the measurement in nano seconds.
// This is the first Keyframe so we need to fix its position.
b.push(Keyframe(0, tracklets, camera, pose_prior.at(0), Keyframe::FixationStatus::Pose));

// If no depth data is available we need to fix scale for second Keyframe.
b.push(Keyframe(1, tracklets, camera, pose_prior.at(1), Keyframe::FixationStatus::Scale));

// All others will have 6 DOFs.
for (int i = 2; i < int(stamps.size()); ++i) {
    b.push(Keyframe(i, tracklets, camera, pose_prior.at(i)));
}

// Run bundle adjuster.
std::string summary = b.solve();
std::cout << summary << std::endl;

// Access optimized poses and print them.
for(const auto & kf_ptrs:b.getActiveKeyframePtrs()){
    std::cout   <<"Keyframe id="<<kf_ptrs.first
                <<"\noptimized keyframe pose=\n"<<kf_ptrs.second->getEigenPose().matrix()
                <<"------------------------------------"<<
                <<std::endl;
}
```

* If you do not want to add all frames as keyframes, you need to do selection, which does KeyframeSelection for you. Have a look at keyframe_selection_schemes for different selection strategies.

```cpp
#include <keyframe_selector.hpp>
#include <keyframe_selection_schemes.hpp>
#include <bundle_adjuster_keyframes.hpp>

using namespace keyframe_bundle_adjustment;
// Get adjuster and camera as before.
BundleAdjusterKeyframes bundle_adjuster;

// This object selects keyframes for you.
KeyframeSelector kf_selector;

// Choose selection scheme and add it to selector.
double time_difference_sec = 0.5; // time lap between frames

KeyframeSelectionSchemeBase::ConstPtr scheme0 =
    std::make_shared<KeyframeSelectionSchemeTime>(time_difference_sec);
kf_selector.addScheme(scheme0);

// Dummy frames, here you have to put the new data.
std::map<KeyframeId, Keyframe> last_frames;
last_frames[0] = Keyframe(0, {}, Camera::Ptr(), Eigen::Isometry3d::Identity());
last_frames[1] = Keyframe(10000, {}, Camera::Ptr(), Eigen::Isometry3d::Identity());

TimestampNSec ts1{10000 + convert(TimestampSec(2. * time_difference_sec))};
Keyframe new_frame0(ts1, {}, Camera::Ptr(), Eigen::Isometry3d::Identity());

TimestampNSec ts2{10000 + convert(TimestampSec(time_difference_sec / 2.))};
Keyframe new_frame1(ts2, {}, Camera::Ptr(), Eigen::Isometry3d::Identity());

// Do selection of frames, by comparing to internal frame buffer.
std::vector<Keyframe> selected_keyframes =
    kf_selector.select({new_frame0, new_frame1}, last_frames);

// Push them to bundle adjuster.
bundle_adjuster.push(selected_keyframes);

// Deactivate keyframes to cut estimation window.
bundle_adjuster.deactivateKeyframes();

// Solve the problem.
std::string summary = bundle_adjuster.solve();
```

* Landmark selection is done inside of BundleAdjusterKeyframes. You can add your own landmark selection scheme to it through the constructor. For different schemes have a look at landmark_selection_schemes.hpp.

## Issues

## Notation

### Transforms and Poses

Throughout the library we use a consitent notation of poses and transforms:

* names consist of *_frameB_frameA
* poses are defined such as a concatenation of all non-inversed poses gives the transform to transform a point p from one cos to another in the following way:

```
p_b = transform_b_c1*transform_c1_c2*transform_c2_a*p_a
```

* in ros notation this corresponds to *_target_source

* this means transforms point from target to source (to origin!) which is against my intuition

* example (2d): p_a=(2,0); p_b=(-2,0)   ->  transform_b_a: zero rotation; translation=(-4,0);

## History

* Unittests work
* Runs on kitti data
* Evaluation on Kitti leads to 13th rank (March 16 2018)
* Tested on Ubuntu 16.04, Ros-Kinetic, Gcc 7.2
* Tested Image of VirtualBox

## Credits

* Johannes Gräter
