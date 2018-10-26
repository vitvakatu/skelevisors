#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import tf2_msgs.msg
import roslib
import math
import geometry_msgs.msg as msgs
import turtlesim.srv
from darwin_gazebo.darwin import Darwin

if __name__ == '__main__':
	rospy.init_node("walker_demo", anonymous=True)
	print('Hello')
	darwin = Darwin()
	rospy.loginfo("Darwin initialization finished")
	print("Hello")
	
	rospy.sleep(1)

	darwin.set_angles({"j_high_arm_l": -2})
	
	rospy.sleep(1)
	print("Hello")

	darwin.set_angles({"j_tilt": 0})

	rospy.sleep(1)
	print("Hello")

	darwin.set_angles({"j_tilt": -1})
	



