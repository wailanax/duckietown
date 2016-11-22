#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import Tau, BoolStamped
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

import pdb

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
        self.image_size = [480,640,3] 
        self.curr_frame = np.zeros(self.image_size, np.uint8)
        self.prev_frame = np.zeros(self.image_size[0:2], np.uint8)
        self.Et_thresh = 0.01;
        #self.hsv_red1 = np.array([120, 70, 90]) #np.array([120, 40, 80])
        #self.hsv_red2 = np.array([179, 160, 140]) #np.array([179, 140, 136])
        #self.hsv_red3 = np.array([165, 140, 100])
        #self.hsv_red4 = np.array([180, 255, 255])
        #self.canny_thresholds = [80,200]
        #self.hough_threshold  = 5 #10 #20
        #self.hough_min_line_length = 5 #8 #3
        #self.hough_max_line_gap = 5 #10
        #self.dilation_kernel_hsv_size = 3
        #self.dilation_kernel_edge_size = 3

        # Publishers
        self.pub_tau = rospy.Publisher("~tau", Tau, queue_size=1)
        self.pub_im_detection = rospy.Publisher("~im_detection", Image, queue_size=1)

        # Subscribers
        # this would require remapping...
        if self.debug: # get image from computer camera
            
            rospy.loginfo("[%s] debug = %s)." %(self.node_name, self.debug) )
            
            self.c0 = [self.image_size[0]/2, self.image_size[1]/2]
            #Open the Video Feed
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print 'Error opening file...'
                return -1
            
            # set up timer to capture image
            rospy.Timer(rospy.Duration(1.0/15), self.cbDebug)
        else:
            self.sub_image = rospy.Subscriber("~image", CompressedImage, self.cbImage, queue_size=1)
            # Determine camera center!
            #self.c0 =  
            
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
        cv2.imshow('image', frame)
        cv2.waitKey(1)

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

        if self.debug:
            # just get the data
            image_raw = image_msg.data
        else:
            # Decode from compressed image using OpenCV
            try:
                image_raw = image_cv_from_jpg(image_msg.data)
            except ValueError as e:
                self.loginfo('Could not decode image: %s ' %e)
                return
            
        # resize to make processing quicker (downsample)
        h_orig, w_orig = image_raw.shape[0:2]
        if self.image_size[0] != h_orig or self.image_size[1] != w_orig:
            image_raw = cv2.resize(image_raw, (self.image_size[0], self.image_size[1]),
                interpolation=cv2.INTER_NEAREST)


        # Lets deal with only one color channel for now...
        # Choose the red channel
        #image_r = image_raw[:,:,2]
        im1 = image_raw[:,:,2]

        # May need to do LPF'ing (block averaging)

        # combine the image with the previous frame
        # CHECK: Need to make sure that order is ok for time derivative...
        #imgseq = cv2.merge((image_r, self.prev_frame)).astype(float)
        im2 = self.prev_frame
        
        #Ey, Ex, Et = gradient(image_time)
        #dx = np.diff(image_time, axis=1)
        #dy = np.diff(image_time, axis=0)
        #dt = np.diff(image_time, axis=2)
        #kernel = 0.25*np.ones([2,2,2])

        #pdb.set_trace()

        # Compute partial derivatives with a 2x2x2 cube.
        # Note right edge will just be zeros...
        Ex = np.zeros(np.array(self.image_size[0:2]))
        Ey = np.zeros(np.array(self.image_size[0:2]))
        Et = np.zeros(np.array(self.image_size[0:2]))
        '''
        for i in range(self.image_size[0]-1):
            for j in range(self.image_size[1]-1):
                k = 0
                Ex[i,j] = 0.25*(imgseq[i,j+1,k] - imgseq[i,j,k]
                                + imgseq[i+1,j+1,k] - imgseq[i+1,j,k]
                                + imgseq[i,j+1,k+1] - imgseq[i,j,k+1]
                                + imgseq[i+1,j+1,k+1] - imgseq[i+1,j,k+1])
                Ey[i,j] = 0.25*(imgseq[i+1,j,k] - imgseq[i,j,k]
                                + imgseq[i+1,j+1,k] - imgseq[i,j+1,k]
                                + imgseq[i+1,j,k+1] - imgseq[i,j,k+1]
                                + imgseq[i+1,j+1,k+1] - imgseq[i,j+1,k+1])
                Et[i,j] = 0.25*(imgseq[i,j,k+1] - imgseq[i,j,k]
                                + imgseq[i+1,j,k+1] - imgseq[i+1,j,k]
                                + imgseq[i,j+1,k+1] - imgseq[i,j+1,k]
                                + imgseq[i+1,j+1,k+1] - imgseq[i+1,j+1,k]) 
        '''
        Ex[1:-1,1:-1] = 0.25*(im1[1:-1, 1:-1] - im1[1:-1, :-2] + im1[:-2,1:-1] - im1[:-2,:-2] \
                        +  im2[1:-1, 1:-1] - im2[1:-1, :-2] + im2[:-2, 1:-1] - im2[:-2,:-2])

        Ey[1:-1,1:-1] = 0.25*(im1[1:-1, 1:-1] - im1[:-2, 1:-1] + im1[1:-1, :-2] - im1[:-2 ,:-2] \
                        +  im2[1:-1, 1:-1] - im2[:-2, 1:-1] + im2[1:-1, :-2] - im2[:-2 ,:-2])

        Et[1:-1, 1:-1] = 0.25*(im1[1:-1, 1:-1] - im2[1:-1,1:-1] + im1[1:-1, :-2] - im2[1:-1, :-2] \
                        +  im1[:-2, 1:-1] - im2[:-2, 1:-1] + im1[:-2, :-2] - im2[:-2,:-2])
        # Threshold Et
        thresh_idx = Et < self.Et_thresh
        Et[thresh_idx] = 0

        # Compute the radial gradient
        x = np.arange(self.image_size[1]) # cols
        y = np.arange(self.image_size[0]) # rows

        xx,yy = np.meshgrid(x,y);
        xx -= self.c0[0] #subtract off the image center.
        yy -= self.c0[1] #subtract off the image center.

        G = (xx*Ex) + (yy*Ey) # These are elementwise!

        # Compute C (inv of TTC)
        #G_squared = G**2 # also elementwise!
        #GEt = G*Et # again elementwise
        C = -1*np.sum(G*Et)/np.sum(G**2)

        if C == 0:
            tau = np.inf
        else:
            tau = 1/C

        # output tau
        tau_msg = Tau()
        tau_msg.tau = tau
        self.pub_tau.publish(tau_msg)

        # update previous frame
        self.prev_frame = im1

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