#!/usr/bin/env python

import rospy
import numpy as np
from duckietown_msgs.msg import LanePose
import csv

# tile length in centimeters
TILE_LENGTH = 60.0

# width across both lanes in centimeters
TOTAL_LANE_WIDTH = 48.0

class PoseEstimator:
    def __init__(self, outputFile):
        f = open(outputFile, 'wt')
        self.writer = csv.writer(f)
        self.oldLP = LanePose()
        self.z = 0.0

        rospy.Subscriber('/teamgrapes/lane_controller_node/lane_pose', LanePose, self.estimatePose)

    def estimatePose(self, lanepose_msg):
        if self.oldLP.header.stamp.to_sec() > 0:
            dt = (lanepose_msg.header.stamp - self.oldLP.header.stamp).to_sec()
            v = self.oldLP.d / np.sin(self.oldLP.phi)
            dz = v * cos(self.oldLP.phi)
            delta_z = dz * dt
            self.z += delta_z

        self.oldLP = lanepose_msg
        rospy.loginfo('current z is: %f', z)

if __name__ == '__main__':
    rospy.init_node('lane_pose_estimator_node', anonymous=True)
    pe = PoseEstimator('./estimatedpose.csv')
    rospy.spin()