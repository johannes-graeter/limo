#!/usr/bin/env python
# In[0]:
# Script for demonstrating feature_tracking
from matplotlib import pyplot as plt
import numpy as np
import pykitti

import feature_tracking_core.tracker_libviso as t

# In[1]:
basedir = '/limo_data/dataset'
sequence = '04'

# In[2]:
# Load the data. Optionally, specify the frame range to load.
# dataset = pykitti.odometry(basedir, sequence)
dataset = pykitti.odometry(basedir, sequence, frames=range(0, 20, 2))

# In[3]:
tracker = t.TrackerLibViso()
for image in dataset.cam0:
    t.push_back(tracker, np.expand_dims(np.asarray(image), axis=-1))

# In[4]:
tracklets = t.get_tracklets(tracker)
print("Number of tracklets={}".format(len(tracklets)))
tracklets_numpy = [np.asarray([(point.p1_.u_, point.p1_.v_) for point in tracklet]) for tracklet in tracklets]

# In[5]:
plt.imshow(dataset.get_cam0(-1), cmap='gray')
for t in tracklets_numpy:
    plt.plot(t[:,0], t[:,1])
