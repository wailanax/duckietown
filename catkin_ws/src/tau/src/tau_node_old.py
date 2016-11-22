#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import TauOld as Tau
from duckietown_msgs.msg import BoolStamped
from duckietown_utils.jpg import image_cv_from_jpg
from sensor_msgs.msg import CompressedImage, Image

import cv2
import numpy as np
import rospy
import threading
import time
import sys
import argparse
import math

class TauNode(object):
    def __init__(self,debug):
        self.node_name = "TauNode"

        self.debug = debug

        # Thread lock
        self.thread_lock = threading.Lock()

        # Constructor of line detector
        self.bridge = CvBridge()

        # Params that won't change
        self.prev_dx = 0;
        self.prev_dy = 0;
        self.prev_tau = 0;
        self.prev_img_time = rospy.Time.now();

        # Params that could change (tie them to a param or a topic)
        self.active = True
        self.verbose = True
        # Right now the joymapper maps to a param for line_detector_node/verbose...
        # figure out if we can remap this or if we need to fix this in joy_mapper_node
        self.image_size = [480,640] 
        self.hsv_red1 = np.array([120, 70, 90]) #np.array([120, 40, 80])
        self.hsv_red2 = np.array([179, 160, 140]) #np.array([179, 140, 136])
        self.hsv_red3 = np.array([165, 140, 100])
        self.hsv_red4 = np.array([180, 255, 255])
        self.canny_thresholds = [80,200]
        self.hough_threshold  = 5 #10 #20
        self.hough_min_line_length = 5 #8 #3
        self.hough_max_line_gap = 5 #10
        self.dilation_kernel_hsv_size = 3
        self.dilation_kernel_edge_size = 3

        # Publishers
        self.pub_tau = rospy.Publisher("~tau", Tau, queue_size=1)
        self.pub_im_detection = rospy.Publisher("~im_detection", Image, queue_size=1)

        # Subscribers
        # this would require remapping...
        if not self.debug:
            self.sub_image = rospy.Subscriber("~image", CompressedImage, self.cbImage, queue_size=1)
        else:
            rospy.loginfo("[%s] debug = %s)." %(self.node_name, self.debug) )
            # set up timer to capture image
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print 'Error opening file...'
                return -1
            cv2.namedWindow('Control', cv2.WINDOW_AUTOSIZE)
            cv2.createTrackbar('LowH',  'Control', self.hsv_red1[0], 179, self.nothing)
            cv2.createTrackbar('HighH', 'Control', self.hsv_red2[0], 179, self.nothing)
            cv2.createTrackbar('LowS',  'Control', self.hsv_red1[1], 255, self.nothing)
            cv2.createTrackbar('HighS', 'Control', self.hsv_red2[1], 255, self.nothing)
            cv2.createTrackbar('LowV',  'Control', self.hsv_red1[2], 255, self.nothing)
            cv2.createTrackbar('HighV', 'Control', self.hsv_red2[2], 255, self.nothing)
            cv2.createTrackbar('HoughThresh', 'Control', self.hough_threshold, 50, self.nothing)
            cv2.createTrackbar('HoughMinLineLength', 'Control', self.hough_min_line_length, 50, self.nothing)
            cv2.createTrackbar('HoughMaxLineGap', 'Control', self.hough_max_line_gap, 50, self.nothing)

            rospy.Timer(rospy.Duration(1.0/15), self.cbDebug)

        #self.sub_april = rospy.Subscriber("~detections", AprilTagDetectionArray, self.cbDetection)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch)

        rospy.loginfo("[%s] Initialized (verbose = %s)." %(self.node_name, self.verbose) )

    def nothing(self,x):
        pass

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data
        # I'm not sure what this does so lets ignore it for now
        # I originally thought this was what turns the image on and off (but thats verbose)
        # If I have a need to turn the whole node on and off may look into this

    def cbDebug(self,event):
        ret, frame = self.cap.read()
        img_msg = Image()
        img_msg.data = frame
        img_msg.header.stamp = event.current_real
        self.cbImage(img_msg)

    def cbImage(self, image_msg):
        # If not active return immediately
        if not self.active:
            return

        # Start a daemon thread to process the image.
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()
        # this function should return quickly since the thread is a daemon

    def processImage(self,image_msg):
        # Acquire the thread lock but do not block for it (False - nonblocking) 
        if not self.thread_lock.acquire(False): 
            # return immediately
            return

        try:
            self.processImage_(image_msg)
        finally:
            # Release the thread lock
            self.thread_lock.release()

    def processImage_(self,image_msg):

        if not self.debug:
            # Decode from compressed image using OpenCV
            try:
                image_cv = image_cv_from_jpg(image_msg.data)
            except ValueError as e:
                self.loginfo('Could not decode image: %s ' %e)
                return
        else:
            image_cv = image_msg.data
            

        # resize to make processing quicker (downsample)
        h_orig, w_orig = image_cv.shape[0:2]
        if self.image_size[0] != h_orig or self.image_size[1] != w_orig:
            image_cv = cv2.resize(image_cv, (self.image_size[0], self.image_size[1]),
                interpolation=cv2.INTER_NEAREST)

        # Do we need to color correct?
        # Lets assume no for now

        # convert to HSV 
        #bgr = np.copy(image_cv)
        hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

        # color threshold to detect red objects
        kernel_hsv = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
            (self.dilation_kernel_hsv_size, self.dilation_kernel_hsv_size))
        kernel_edge = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
            (self.dilation_kernel_edge_size, self.dilation_kernel_edge_size))
        if self.debug:
            self.hsv_red1[0] = cv2.getTrackbarPos('LowH', 'Control')
            self.hsv_red1[1] = cv2.getTrackbarPos('LowS', 'Control')
            self.hsv_red1[2] = cv2.getTrackbarPos('LowV', 'Control')
            self.hsv_red2[0] = cv2.getTrackbarPos('HighH', 'Control')
            self.hsv_red2[1] = cv2.getTrackbarPos('HighS', 'Control')
            self.hsv_red2[2] = cv2.getTrackbarPos('HighV', 'Control')
            bw = cv2.inRange(hsv, self.hsv_red1, self.hsv_red2)
            bw = cv2.dilate(bw, kernel_hsv)
            bw = cv2.erode(bw, kernel_hsv) 

        else:
            bw1 = cv2.inRange(hsv, self.hsv_red1, self.hsv_red2)
            bw2 = cv2.inRange(hsv, self.hsv_red3, self.hsv_red4)
            bw = cv2.bitwise_or(bw1, bw2)
        

        # also perform canny edge detection
        gray = cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, self.canny_thresholds[0], self.canny_thresholds[1], apertureSize = 3) 
        
        edges = cv2.dilate(edges, kernel_edge)

        # get only red edges
        edge_red = cv2.bitwise_and(bw, edges)

        # detect lines
        if self.debug:
            self.hough_threshold = cv2.getTrackbarPos('HoughThresh', 'Control')
            self.hough_min_line_length = cv2.getTrackbarPos('HoughMinLineLength', 'Control')
            self.hough_max_line_gap = cv2.getTrackbarPos('HoughMaxLineGap', 'Control')
        
        # (input_edge_img, rho, theta, thresh, minLinLength,maxLineGap)
        # input_edge_img - the edge detected image as input
        # rho - the resolution of the parameter r in pixels (1 pixel)
        # theta - the resolution of the parameter theta in radians (1 deg)
        # thresh - the minimum number of intersections to detect a line
        # minLinLength - minimum number of points that can form a line
        # maxLineGap - maximum gap between 2 points to be considered same line
        lines = cv2.HoughLinesP(edge_red, 1, np.pi/180, 
            self.hough_threshold, np.empty(1), 
            self.hough_min_line_length, 
            self.hough_max_line_gap)

        lineimg = np.zeros([480,640,3], np.uint8)
        if lines is not None:
            for x1,y1,x2,y2 in lines[0]:
                cv2.line(lineimg,(x1,y1),(x2,y2),(0,255,0),5)
            
            #print "line: "
            #print lines[0]
            lines_min = lines.min(1)[0]
            lines_max = lines.max(1)[0]
            #print lines_min
            #print lines_min[0]
            #print lines_max
            x_max = max(lines_max[0], lines_max[2])
            x_min = min(lines_min[0], lines_min[2])
            y_max = max(lines_max[1], lines_max[3])
            y_min = min(lines_min[1], lines_min[3])
            
            # use that to compute the length.
            dx = float(x_max - x_min)
            dy = float(y_max - y_min)

            # compute time step
            img_time = image_msg.header.stamp
            time_diff = img_time - self.prev_img_time
            ts = time_diff.to_sec()

            # compute derivatives
            dx_dot = (dx - self.prev_dx)/ts
            dy_dot = (dy - self.prev_dy)/ts

            # compute tau
            if dx_dot == 0:
                taux = np.inf
            else:
                taux = dx / dx_dot

            if dy_dot == 0:
                tauy = np.inf
            else: 
                tauy = dy / dy_dot

            tau = np.array([taux, tauy])

            # save values for next iteration
            self.prev_dx = dx
            self.prev_dy = dy
            self.prev_img_time = img_time
            self.prev_tau = tau
        else:
            tau = self.prev_tau
            
        if self.debug:
            cv2.imshow('image', image_cv)
            cv2.imshow('bw', bw)
            cv2.imshow('edges',edges)
            cv2.imshow('red', edge_red)
            cv2.imshow('lineimg', lineimg)
            cv2.waitKey(1)

        # output our detected image for debugging
        detection_img = self.bridge.cv2_to_imgmsg(lineimg, "bgr8")
        detection_img.header.stamp = image_msg.header.stamp
        self.pub_im_detection.publish(detection_img)

        # output tau
        tau_msg = Tau()
        tau_msg.tau = tau
        self.pub_tau.publish(tau_msg)

    def onShutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("Shutdown.")

if __name__ == '__main__':
    rospy.init_node('tau', anonymous=False)
    argv=rospy.myargv(argv=sys.argv[1:])

    parser = argparse.ArgumentParser(
        description='Initialize the Tau Node')
    parser.add_argument('--debug',action='store_true',
        help='Use Debug mode (works on laptop)')
    parser.add_argument('args', nargs=argparse.REMAINDER)

    args = parser.parse_args(argv)
    
    debug = args.debug

    tau_node = TauNode(debug)
    rospy.on_shutdown(tau_node.onShutdown)
    rospy.spin()