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
        self.K = np.matrix(self.K, dtype=np.float32)
        self.K = K.reshape((3,3))
        self.R = ci.R
        self.P = ci.P
        self.width = ci.width
        self.height = ci.height
        self.focal = self.K[0]
        self.pp = (self.K[2], self.K[5])
        self.camParamsSet = True
    
    # finds the essential matrix using K and the fundamental matrix
    def findEssentialMat(self):
        # the essential matrix is equal to the K_T * F * K where K_T is the transpose of K
        # E = self.K.T * np.mat(F) * self.K

        # E = np.mat(F)

        # normalize image coordinates
        normOldPts = cv2.undistortPoints(self.oldFeatures, self.K)
        normNewPts = cv2.undistortPoints(self.newFeatures, self.K)

        E, mask = cv2.findFundamentalMat(normOldPts, normNewPts, method=cv2.FM_RANSAC)

        return E, normOldPts, normNewPts, mask

    # finds R and t, the rotation and translation between two camera angles.
    def recoverPose(self):
        E, normOldPts, normNewPts, mask = self.findEssentialMat()
        w, u, vt = cv2.SVDecomp(np.mat(E))

        # this provides us with 4 possible solutions. Return the best one:
        if np.linalg.det(u) < 0:
            u *= -1.0
        if np.linalg.det(vt) < 0:
            vt *= -1.0

        # solve for R and t using algorithm from Hartley & Zisserman
        W=np.mat([[0,1,0],[-1,0,0],[0,0,1]], dtype=np.float32)
        Ra = np.mat(u) * W * np.mat(vt)
        Rb = np.mat(u) * W.T * np.mat(vt)
        t = u[:,2]

        # we now have four solutions
        Pa = np.concatenate((Ra, t), axis=1)
        Pb = np.concatenate((Ra, -1.0*t), axis=1)
        Pc = np.concatenate((Rb, t), axis=1)
        Pd = np.concatenate((Rb, -1.0*t), axis=1)

        # solve for true solution by testing cheirality of each option

        # use only inliers to test cheirality
        maskedOldFeatures = np.array(self.oldFeatures[mask == 1], dtype=np.float32)
        maskedNewFeatures = np.array(self.newFeatures[mask == 1], dtype=np.float32)
        # maskedOldFeatures = np.array(normOldPts[mask == 1], dtype=np.float32)
        # maskedNewFeatures = np.array(normNewPts[mask == 1], dtype=np.float32)

        # we assume the perspective matrix for our old frame is P = [I | 0]
        Po = np.mat([[1,0,0,0],[0,1,0,0],[0,0,1,0]], dtype=np.float32)
                                            # 2xN array
        p4d = cv2.triangulatePoints(Po, Pa, maskedOldFeatures[:1].T, maskedNewFeatures[:1].T)
        Q = p4d[:,0]

        v = vt.T
        v_2 = -2*v
        Ht = np.mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[v_2[0,2], v_2[1,2], v_2[2,2]]])

        # if c1 < 0, the point is behind the first camera
        c1 = Q[2] * Q[3]

        # if c2 < 0, the point is behind the second camera
        c2 = (Pa * Q)[2] * Q[3]

        c3 = Q[2] * (Ht * Q)[3]

        # if both c1 and c2 are greater than 0, Pa is our solution
        if (c1 > 0 and c2 > 0):
            return Pa[:,:3], Pa[:,3]

        if (c1 < 0 and c2 < 0):
            return Pb[:,:3], Pb[:,3]

        if (c1*c2 < 0 and c3 > 0):
            return Pc[:,:3], Pc[:,3]

        return Pd[:,:3], Pd[:,3]
        
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
        undistorted = cv2.undistort(undistorted, self.K, self.D)

        # convert image to grayscale
        undistorted = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

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
            # F, mask = cv2.findFundamentalMat(self.newFeatures, self.oldFeatures)
            # _, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, self.newFeatures, self.oldFeatures, focal=self.focal, pp=self.pp)
            # E, normOldPts, normNewPts, mask = self.findEssentialMat()
            self.cur_R, self.cur_t = self.recoverPose()

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
        
        print('num features %d' % (self.newFeatures.shape[0]))
        if (self.newFeatures.shape[0] < self.minNumFeatures):
            self.newFeatures = self.detector.detect(self.newImage)
            self.newFeatures = np.array([x.pt for x in self.newFeatures], dtype=np.float32)

        self.oldFeatures = self.newFeatures
        self.oldImage = self.newImage.copy()
        #kpim = self.oldImage.copy()
        #for feature in self.oldFeatures:
            #print(feature)
         #   cv2.circle(kpim, (int(feature[0]), int(feature[1])), 3, (0,0,255))
        #cv2.imwrite('odometryimage%d.png' % (self.imindex), kpim)
        #self.imindex += 1
        self.publishPose()

if __name__ == '__main__':
    rospy.init_node('heatmap_odometry_node', anonymous=False)
    vo = VisualOdometry('./odometry.csv')
    rospy.spin()
