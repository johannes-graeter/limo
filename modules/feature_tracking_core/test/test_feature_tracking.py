#!/usr/bin/env python2
import os
import unittest

import cv2
import feature_tracking_core.tracker_libviso as t
import numpy as np

class TestFeatureTracking(unittest.TestCase):
    def setUp(self):
        self.tracker = t.TrackerLibViso()
        test_dir = os.path.dirname(os.path.realpath(__file__)) 
        self.images = [cv2.imread(os.path.join(test_dir, image_name),0) for image_name in ["000106.png", "000107.png"]]

    def test_feature_tracking(self):
         for image in self.images:
             t.push_back(self.tracker, np.expand_dims(image, axis=-1))

         tracklets = t.get_tracklets(self.tracker)
         print("Number of tracklets={}".format(len(tracklets)))
         self.assertEqual(len(tracklets), 3138)

if __name__ == '__main__':
    unittest.main()
