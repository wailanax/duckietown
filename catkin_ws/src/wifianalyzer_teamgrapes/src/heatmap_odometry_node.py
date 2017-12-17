import cv2
import numpy as np
import sys
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

#class PinholeCamera:
#    def __init__(self, width, height, fx, fy, cx, cy, k1, k2, p1, p2, k3):
#        self.width = width
#        self.height = height
#        self.fx = fx
#        self.cx = cx

class VisualOdometry:
    def __init__(self):
        self.camParamsSet = False
        self.xPos = 0.0
        self.yPos = 0.0
        self.zPos = 0.0
        self.lk_params = dict(winsize=(15,15), criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        
        self.detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
        
        rospy.subscriber('/teamgrapes/camera_node/camera_info', CameraInfo, self.setCamParams)
        rospy.subscriber('/teamgrapes/camera_node/image/compressed', CompressedImage, self.computePose)
        
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
    
    # calculate the movement of features between frames using KLT optical flow
    def trackFeatures(self):
        kp, st, err = cv2.calcOpticalFlowPyrLK(self.oldImage, self.newImage, self.oldFeatures, **(self.lk_params))
        st = st.reshape(st.shape[0])
        self.oldFeatures = self.oldFeatures[st == 1]
        self.newFeatures = kp[st == 1]
        
    def computePose(self, image_msg):
        # if we haven't set the camera parameters, do nothing
        if (not self.camParamsSet):
            return
        
        # if we haven't processed the first image, do that
        if (self.firstImage):
            self.oldImage = image_cv_from_jpg(image_msg.data)
            self.oldFeatures = self.detector.detect(self.oldImage)
            self.oldFeatures = np.array([x.pt for x in self.oldFeatures], dtype=np.float32)
            self.firstImage = False
            self.publishPose()
            return
        
        if (self.secondImage):
            self.newImage = image_cv_from_jpg(image_msg.data)
            self.trackFeatures()
            E, mask = cv2.findEssentialMat(self.newFeatures, self.oldFeatures, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            _, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, self.newFeatures, self.oldFeatures, focal=self.focal, pp=self.pp)
            self.oldFeatures = self.newFeatures
            self.oldImage = self.newImage
            self.secondImage = False
            return
                
        # process subsequent images
        self.newImage = image_cv_from_jpg(image_msg.data)
        self.trackFeatures()
        E, mask = cv2.findEssentialMat(

if __name__ == '__main__':
    rospy.init_node('heatmap_odometry_node', anonymous=False)
    vo = VisualOdometry()
    rospy.spin()
