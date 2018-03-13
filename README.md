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
* Detailed install instructions and examples will follow. 

## Installation

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

### General

* [ ] optimizer too slow, (1.5 sec with 10 keyframes)
  * [X] jacobian eval takes long -> yes
    * [X] why? -> many evaluations with autodiff
    * [ ] use analytic diff?
  * [X] how many LMs do i need so that I am real time capable?
    * since I only do it on keyframes, 500ms is ok -> 500 parameter blocks(450 landmarks, 50 keyframes),  1700(1600) (effective) parameters.
    * results are on dell laptop with autodiff and 5 point as prior
  * [X] try different optimizer
    * DENSE_SCHUR seems to be the one according to ceres docu
    * Tiny solver could be more effective but is still experimental
  * [ ] tune params

* [ ] landmark selection
  * [ ] categorize landmarks as near middle and far field
  * [ ] select landmarks with "flow fields"

* [X] throws an error after 20 frames

* [ ] how fast is framework (all but optimization)?

### Lidar

* [ ] Add depth data to framework
  * [ ] make new class LidarKeyframe: public Keyframe and add depth data associated by landmarks
  * [ ] overload BundleAdjuster::push with LidarKeyframe
    * [ ] convert to old Keyframes (cast!)
    * [ ] call old push to triangulate
    * [ ] add depth if landmark if it is not in far-field
    * [ ] @TODO(Johannes) move landmark selection to push

* [ ] adapt unittests

* [ ] write tool in keyframe_bundle_adjustment_ros_tool

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

## Credits

* Johannes Gräter
