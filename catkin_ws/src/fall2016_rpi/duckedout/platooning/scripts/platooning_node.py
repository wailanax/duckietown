import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

def platooning():

	rospy.init_node('platooning', anonymous=False)
	rospy.Subscriber("~tag_pose", PoseStamped, callback)

def callback(posedata):

	#Somehow we want to take the pose data and turn it into a desired pose here
	#i.e. tag is skewed, move move and adjust pose so that you face tag head on
	#at set distance away.
