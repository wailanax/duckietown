#!/usr/bin/env python

import rospy
import math
from duckietown_msgs.msg import Twist2DStamped
from wifianalyzer_teamgrapes import wifiutils as wu
xPos = 0.0
yPos = 0.0
theta = 0.0
curTime = None

def updatePosition(twist_msg):
    global xPos, yPos, theta, curTime
    linearVel = twist_msg.v
    angularVel = twist_msg.omega
    dt = twist_msg.header.stamp.to_sec() - curTime.to_sec()
    curTime = twist_msg.header.stamp
    
    theta = theta + angularVel*dt
    xPos = math.cos(theta)*linearVel*dt + xPos
    yPos = math.sin(theta)*linearVel*dt + yPos
    
    strength = wu.get_duckietown_strength()
    
    rospy.loginfo(rospy.get_caller_id() + " robot is now at x:%f y:%f theta:%f with strength %d", xPos, yPos, theta, strength)

# listens for updates to the lane_pose topic and updates known position accordingly
def listener():
    print "listening\n"
    #rospy.init_node('heatmap_data_node', anonymous=True)
    rospy.Subscriber("/teamgrapes/lane_controller_node/car_cmd", Twist2DStamped, updatePosition)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('heatmap_data_node', anonymous=True)
    curTime = rospy.get_rostime()
    print "starting\n"
    listener()
