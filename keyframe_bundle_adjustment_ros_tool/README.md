# Keyframe bundle adjustment ros tool

* This tool uses keyframe_bundle_adjustment to make postprocessing for visual odometry
* In contrast to existing tools such as orbslam it is meant ot be very modular so you can try different prior estimation methods or scale estimation yourself
* Nodes:
  * Mono standalone: input: tracklets from 1 camera; use 5 point as prior and do pure mono odometry without scale
  * Mono with motion prior: input: tracklets from 1 camera, scaled motion constraint; use motion constraint as prior and smooth trajectory, scale is not fixed but regularized in whole window
  * Multicam with motion prior: input: tracklets from multi camera, scaled motion constraint; smooth with multiple cameras
  * Mono/Mutlicam with lidar: input: tracklets with depth from 1/multi camera; use depth from lidar to estimate scale (Alex)

## Installation

standard

## Usage

## History
DONE:
* Mono standalone

TODO:

* Mono with lidar (15.1.2018)
* Mono with motion (due 24.11.)
* Multicam with motion prior (due ????)
* Multi with lidar (due ????)

## Credits

* Johannes Gräter
* Alexander Wilczynski (mono lidar)
