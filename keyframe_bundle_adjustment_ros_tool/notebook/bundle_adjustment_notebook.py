#!/usr/bin/env python
# In[0]:
# Script for demonstrating feature_tracking
from matplotlib import pyplot as plt
import numpy as np
import itertools
import pykitti

import feature_tracking_core.tracker_libviso as t

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
camera_data.focal_length = dataset.calib["K_cam0"][0,0]
camera_data.cx = dataset.calib["K_cam0"][0,-1]
camera_data.cy = dataset.calib["K_cam0"][1,-1]
camera_data.transform_camera_vehicle = np.eye(4)
camera_data.transform_camera_vehicle[:3,:4] = dataset.calib["T_cam0_velo"]

# In[5]:
bundle_adjuster = kfba.BundleAdjusterKeyframes()
kfba.init_bundle_adjuster(bundle_adjuster, config)

# In[6]:
keyframe_selector = kfba.KeyframeSelector()
kfba.init_keyframe_selector(keyframe_selector, config)

# In[7]:
tracker = t.TrackerLibViso()

# In[8]:
# Use itertools for iteration to preserve generator for data loading.
# In python3 zip does that out of the box.
for image, timestamp_sec in itertools.izip(dataset.cam0, dataset.timestamps):
    t.push_back(tracker, np.expand_dims(np.asarray(image), axis=-1))

    tracklets = t.get_tracklets(tracker)
    print("Number of tracklets={}".format(len(tracklets)))
    tracklets_numpy = [np.asarray([(point.p1_.u_, point.p1_.v_) for point in tracklet]) for tracklet in tracklets]

    plt.imshow(dataset.get_cam0(-1), cmap='gray')
    for t in tracklets_numpy:
        plt.plot(t[:,0], t[:,1])

    kfba.execute_mono_bundle_adjustment(bundle_adjuster, tracklets_numpy, keyframe_selector, camera_data, timestamp_sec)

