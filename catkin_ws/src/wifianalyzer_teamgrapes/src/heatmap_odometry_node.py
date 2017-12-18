#!/usr/bin/env python

import rospy
import cv2
import csv
import numpy as np
import sys
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from duckietown_utils.jpg import image_cv_from_jpg

#class PinholeCamera:
#    def __init__(self, width, height, fx, fy, cx, cy, k1, k2, p1, p2, k3):
#        self.width = width
#        self.height = height
#        self.fx = fx
#        self.cx = cx

class VisualOdometry:
    def __init__(self, outputFile):
        self.camParamsSet = False
        self.firstImage = True
        self.secondImage = True
        self.imindex = 0

        self.f = open(outputFile, 'wt')
        self.writer = csv.writer(self.f)
        self.writer.writerow(('x','y','z'))

        self.minNumFeatures = 500
        self.lk_params = dict(winSize=(15,15), criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        
        self.detector = cv2.FastFeatureDetector(threshold=25, nonmaxSuppression=True)
        
        rospy.Subscriber('/teamgrapes/camera_node/camera_info', CameraInfo, self.setCamParams)
        rospy.Subscriber('/teamgrapes/camera_node/image/compressed', CompressedImage, self.computePose)
        
    def setCamParams(self, ci):
        self.D = ci.D
        self.K = ci.K
        self.R = ci.R
        self.P = ci.P
        self.width = ci.width
        self.height = ci.height
        self.focal = self.K[0]
        self.pp = (self.K[2], self.K[5])
        self.camParamsSet = True
    
    # finds the essential matrix using K and the fundamental matrix
    def findEssentialMat(self, F):
        # reshape K to a 3x3 camera matrix
        K = np.matrix(self.K)
        K = K.reshape((3,3))

        # the essential matrix is equal to the K_T * F * K where K_T is the transpose of K
        E = K.T * np.mat(F) * K

        return E

    # finds R and t, the rotation and translation between two camera angles.
    def recoverPose(self, E):
        w, u, vt = cv2.SVDecomp(np.mat(E))

        # this provides us with 4 possible solutions. Return the best one:
        if np.linalg.det(u) < 0:
            u *= -1.0
        if np.linalg.det(vt) < 0:
            vt *= -1.0

        # solve for R and t using algorithm from Hartley & Zisserman
        W=np.mat([[0,-1,0],[1,0,0],[0,0,1]],dtype=float)
        R = np.mat(u) * W * np.mat(vt)
        t = u[:,2]

        return R, t
        
    # calculate the movement of features between frames using KLT optical flow
    def trackFeatures(self):
        kp, st, err = cv2.calcOpticalFlowPyrLK(self.oldImage, self.newImage, self.oldFeatures, **(self.lk_params))
        st = st.reshape(st.shape[0])
        self.oldFeatures = self.oldFeatures[st == 1]
        self.newFeatures = kp[st == 1]

    def publishPose(self):
        print(self.cur_t)
        x, y, z = (self.cur_t[0,0], self.cur_t[0,1], self.cur_t[0,2])
        rospy.loginfo(rospy.get_caller_id() + " robot is now at x:%f y:%f z:%f", x, y, z)
        self.writer.writerow((x,y,z))

    def computePose(self, image_msg):
        # if we haven't set the camera parameters, do nothing
        if (not self.camParamsSet):
            return
        
        # undistort image
        undistorted = image_cv_from_jpg(image_msg.data)
        K = np.matrix(self.K)
        K = K.reshape((3,3))
        undistorted = cv2.undistort(undistorted, K, self.D)

        # if we haven't processed the first image, do that
        if (self.firstImage):
            self.oldImage = undistorted
            self.oldFeatures = self.detector.detect(self.oldImage)
            self.oldFeatures = np.array([x.pt for x in self.oldFeatures], dtype=np.float32)
            self.firstImage = False
            return
        
        if (self.secondImage):
            self.newImage = undistorted

            self.trackFeatures()
            F, mask = cv2.findFundamentalMat(self.newFeatures, self.oldFeatures)
            # _, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, self.newFeatures, self.oldFeatures, focal=self.focal, pp=self.pp)
            E = self.findEssentialMat(F)
            self.cur_R, self.cur_t = self.recoverPose(E)

            self.oldFeatures = self.newFeatures
            self.oldImage = self.newImage

            self.secondImage = False

            self.publishPose()
            return
            
        # process subsequent images
        self.newImage = undistorted

        self.trackFeatures()
        F, mask = cv2.findFundamentalMat(self.newFeatures, self.oldFeatures)
        # _, R, t, mask = cv2.recoverPose(E, self.newFeatures, self.oldFeatures, focal=self.focal, pp=self.pp)
        E = self.findEssentialMat(F)
        R, t = self.recoverPose(E)

        self.cur_t = self.cur_t + self.cur_R.dot(t)
        self.cur_R = self.cur_R.dot(R)

        if (self.newFeatures.shape[0] < self.minNumFeatures):
            self.newFeatures = self.detector.detect(self.newImage)
            self.newFeatures = np.array([x.pt for x in self.newFeatures], dtype=np.float32)

        self.oldFeatures = self.newFeatures
        self.oldImage = self.newImage
        kpim = self.oldImage.copy()
        for feature in self.oldFeatures:
            kpim = cv2.circle(kpim, (feature[0], feature[1]), 3, (0,0,255))
        cv2.imwrite('odometryimage%d.png' % (self.imindex), kpim)
        self.imindex += 1
        self.publishPose()

if __name__ == '__main__':
    rospy.init_node('heatmap_odometry_node', anonymous=False)
    vo = VisualOdometry('./odometry.csv')
    rospy.spin()
