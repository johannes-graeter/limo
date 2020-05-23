#!/usr/bin/env python
# In[0]:
# Script for demonstrating feature_tracking
from matplotlib import pyplot as plt
import numpy as np
import itertools
import pykitti

import feature_tracking_core.tracker_libviso as tracker_libviso
from matches_msg_types.matches_msg as matches_msg

import keyframe_bundle_adjustment_ros_tool.keyframe_bundle_adjustment_mono as kfba

# In[1]:
basedir = '/limo_data/dataset'
sequence = '04'

# In[2]:
config  = kfba.Config()
config.height_over_ground = 1.73
config.time_between_keyframes_sec = 0.1
config.min_median_flow = 8.0
config.critical_rotation_difference = 0.08
config.max_number_landmarks_near_bin = 300
config.max_number_landmarks_middle_bin = 300
config.max_number_landmarks_far_bin = 300
config.robust_loss_reprojection_thres = 1.0
config.outlier_rejection_quantile = 0.95
config.outlier_rejection_num_iterations = 1
config.max_solver_time = 0.2
config.outlier_labels_yaml = "../res/outlier_labels.yaml"

# In[3]:
# Load the data. Optionally, specify the frame range to load.
dataset = pykitti.odometry(basedir, sequence)

# In[4]:
camera_data  = kfba.CameraData()
camera_data.focal_length = dataset.calib.K_cam2[0,0]
camera_data.cx = dataset.calib.K_cam2[0,-1]
camera_data.cy = dataset.calib.K_cam2[1,-1]
transform_camera_vehicle = dataset.calib.T_cam2_velo

# In[5]:
tracker = tracker_libviso.TrackerLibViso()

def convert_tracklets(tracklets, timestamps):
    msg_tracklets = matches_msg.Tracklets()
    for tracklet in tracklets:
        msg_tracklet = matches_msg.Tracklet()
        for match in tracklet:
            msg_tracklet.feature_points.append(matches_msg.FeaturePoint(match.p1_.u_, match.p1_.v_))

        msg_tracklet.id = tracklet.id_
        msg_tracklets.tracks.append(msg_tracklet)

    max_length = np.max([len(t.feature_points) for t in msg_tracklets.tracks])
    for s in timestamps[:-max_length]:
        msg_tracklets.stamps.append(int(s.total_seconds()*1e9))
    return msg_tracklets

def plot_image(image, tracklets):
    tracklets_numpy = [np.asarray([(point.p1_.u_, point.p1_.v_) for point in tracklet]) for tracklet in tracklets]
    plt.imshow(image)
    for t in tracklets_numpy:
        plt.plot(t[:,0], t[:,1])

# In[6]:
keyframe_selector = kfba.KeyframeSelector()
kfba.init_keyframe_selector(keyframe_selector, config)

# In[7]:
bundle_adjuster = kfba.BundleAdjusterKeyframes()
kfba.init_bundle_adjuster(bundle_adjuster, config)

# Use itertools for iteration to preserve generator for data loading.
# In python3 zip does that out of the box.
timestamps = []
for image, timestamp in itertools.izip(dataset.cam2, dataset.timestamps):
    timestamps.append(timestamp)
    tracker_libviso.push_back(tracker, np.asarray(image))

    tracklets = tracker_libviso.get_tracklets(tracker)
    print("Number of tracklets={}".format(len(tracklets)))

    if len(tracklets) == 0:
        continue
    converted_tracklets = convert_tracklets(tracklets, timestamps)
    kfba.execute_mono_bundle_adjustment(bundle_adjuster, converted_tracklets, keyframe_selector, camera_data, transform_camera_vehicle, timestamp.total_seconds(), config)
    print(bundle_adjuster.keyframes[-1].pose)
    print(bundle_adjuster.keyframes[-1].plane)
