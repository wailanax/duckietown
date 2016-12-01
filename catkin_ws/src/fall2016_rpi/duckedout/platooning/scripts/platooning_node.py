#!/usr/bin/env python

import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose, AprilTagDetectionArray
import numpy
from geometry_msgs.msg import PoseStamped

class PlatoonNode(object):
        def __init__(self):
                self.node_name = rospy.get_name()

                self.kd = 45
                self.kphi = 6

                self.currentv = 0
                self.currentomega = 0
                self.prevz = 100000

                self.pub_vel = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
                self.sub_lane = rospy.Subscriber("~lane_pose", LanePose, self.lane_callback, queue_size=1)
                self.sub_tag = rospy.Subscriber("~apriltags_in", AprilTagDetectionArray, self.tag_callback, queue_size=1)

                rospy.loginfo("[%s] Initilized", self.node_name)

        def lane_callback(self, lane_pose):
                d = lane_pose.d
                phi = lane_pose.phi
                kd = self.kd
                kphi = self.kphi

                if self.currentv == 0:
                        omega_cmd = 0
                else:
                        omega_cmd = -1*kphi*phi - kd*(d + .01)

                lane_msg = Twist2DStamped()
                lane_msg.v = self.currentv
                lane_msg.omega = omega_cmd
                self.pub_vel.publish(lane_msg)

        def tag_callback(self, msg):
                for tag in msg.detections:
                        tag_position = tag.pose.pose.position

                        newz = tag_position.z

                        if newz < .2 and newz > -.2:
                                v_cmd = 0
                        elif newz < self.prevz:
                                v_cmd = .2
                        else:
                                v_cmd = -.2

	                self.currentv = v_cmd

if __name__ == '__main__':
        rospy.init_node('platooning_node', anonymous=False)
        position_controller_node = PlatoonNode()
        rospy.spin()

