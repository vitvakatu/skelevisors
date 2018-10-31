#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import tf2_msgs.msg
import roslib
import math
import geometry_msgs.msg as msgs
import turtlesim.srv
from darwin_gazebo.darwin import Darwin
import vectormath as v

if __name__ == '__main__':
	rospy.init_node("walker_demo", anonymous=True)
	print('Hello')
	darwin = Darwin()
	rospy.loginfo("Darwin initialization finished")
	print("Hello")

	while True:
		x = input()
		y = input()
		z = input()
		if x == 555:
			break
		darwin.set_angles({"j_shoulder_l": x})
		darwin.set_angles({"j_high_arm_l": y})
		darwin.set_angles({"j_low_arm_l": z})

	



