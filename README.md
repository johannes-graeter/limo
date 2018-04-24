# Keyframe bundle adjustment

* This is a library for doing bundle adjustment with visual sensors
* It is supposed to be an add-on module to do temporal inference of the optimization graph in order to smooth the result
* In order to do that online a windowed approach is used
* Keyframes are instances in time which are used for the bundle adjustment, one keyframe may have several cameras (and therefore images) associated with it
* The selection of Keyframes tries to reduce the amount of redundant information while extending the time span covered by the optimization window to reduce drift
* Methodologies for Keyframe selection:
  * Difference in time
  * Difference in motion

* We use this library for combining Lidar with monocular vision.

## Note

This is work in progress, detailed install instructions and examples will follow.

## Installation

### Requirements

* Eigen3: <code>sudo apt-get install libeigen3-dev</code>

* catkin: follow the instructions on [http://wiki.ros.org/catkin](http://wiki.ros.org/catkin) or install ros

* ceres: follow the instructions on [http://ceres-solver.org/installation.html](http://ceres-solver.org/installation.html)
* googletest for unittests: <code>sudo apt-get install libgtest-dev</code>

### Build

* initiate a catkin workspace:
    *<code>cd *your_catkin_workspace*</code>
    *<code>cd *your_catkin_workspace*/src</code>
    *<code>catkin_init_workspace</code>

* clone mrt_cmake_modules into src of workspace:
    *<code>cd *your_catkin_workspace*/src</code>
    *<code>git clone https://github.com/KIT-MRT/mrt_cmake_modules.git</code>

* clone momo into src of workspace:
    *<code>cd *your_catkin_workspace*/src</code>
    *<code>git clone https://github.com/johannes-graeter/momo.git</code>

* build it with catkin:
    *<code>cd *your_catkin_workspace*</code>
    *<code>catkin_make</code>

* unittests:
    *uncomment **<test_depend>gtest</test_depend>** in package.xml to activate unittests
    *<code>cd *your_catkin_workspace*</code>
    *<code>catkin_make run_tests</code>

* tested with docker ros image

standard mrt procedure

## Usage

* bundle_adjuster_keyframes does bundle adjustment on keyframes, triangulates landmarks if needed

```cpp
Example
```

* You need to add Keyframes to bundle_adjuster to add data

```cpp
Example
```

* If you do not want to add all frames as keyframes, you need to do selection, which does KeyframeSelection for you

```cpp
#include <keyframe.hpp>
#include <keyframe_selector.hpp>
#include <keyframe_selection_schemes.hpp>

using namespace keyframe_bundle_adjustment;
// This object selects keyframes for you
KeyframeSelector kf_selector;

// choose selection scheme
double time_difference_sec = 0.5; // time lap between frames

// add it to selector
KeyframeSelectionSchemeBase::ConstPtr scheme0 =
    std::make_shared<KeyframeSelectionSchemeTime>(time_difference_sec);
kf_selector.addScheme(scheme0);

// dummy frames, here you have to put the new data
std::map<KeyframeId, Keyframe> last_frames;
last_frames[0] = Keyframe(0, {}, Camera::Ptr(), Eigen::Isometry3d::Identity());
last_frames[1] = Keyframe(10000, {}, Camera::Ptr(), Eigen::Isometry3d::Identity());

TimestampNSec ts1{10000 + convert(TimestampSec(2. * time_difference_sec))};
Keyframe new_frame0(ts1, {}, Camera::Ptr(), Eigen::Isometr  y3d::Identity());

TimestampNSec ts2{10000 + convert(TimestampSec(time_difference_sec / 2.))};
Keyframe new_frame1(ts2, {}, Camera::Ptr(), Eigen::Isometry3d::Identity());

// do selection of frames, by comparing to internal frame buffer
std::vector<Keyframe> selected_keyframes =
    kf_selector.select({new_frame0, new_frame1}, last_frames);

// push them to bundle adjuster
bundle_adjuster.push(selected_keyframes);

// deactivate keyframes to cut estimation window
bundle_adjuster.deactivateKeyframes();

// solve the problem
auto summary = bundle_adjuster.solve();

```

* TODO: SNIPPET
* corresponding tool: keyframe_bundle_adjustment_ros_tool

## Issues

## Notation

### Transforms and Poses

Throughout the library we use a consitent notation of poses and transforms:

* names consist of *_frameB_frameA
* poses are defined such as a concatenation of all non-inversed poses gives the transform to transform a point p from one cos to another in the following way:

```latex
p_b = transform_b_c1*transform_c1_c2*transform_c2_a*p_a
```

* in ros notation this corresponds to *_target_source

* this means transforms point from target to source (to origin!) which is against my intuition

* example (2d): p_a=(2,0); p_b=(-2,0);transform_b_a: zero rotation translation=(-4,0);

## History

* Unittests work
* Runs on kitti data
* Evaluation on Kitti leads to 13th rank (March 16 2018)

## Credits

* Johannes Gräter
