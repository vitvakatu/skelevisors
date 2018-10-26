#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import tf2_msgs.msg
import roslib
import math
import geometry_msgs.msg as msgs
import turtlesim.srv
from darwin_gazebo.darwin import Darwin

hy = 0.0
lhy = 0.0


def callback(data, darwin):
	global hy
	global lhy
	#darwin = Darwin()
	#print('darwindurak')
	for tr in data.transforms:
		if tr.child_frame_id.startswith('head_'):
			hy = tr.transform.translation.y
		if tr.child_frame_id.startswith('left_hand_'):
			lhy = tr.transform.translation.y
	print(lhy)
	print(hy)
	if lhy > hy:
		darwin.set_angles({"j_pan":1})
		print('+')
	else:
		darwin.set_angles({"j_pan":-1})
		print('-')
		

if __name__ == '__main__':
	rospy.init_node("walker_demo", anonymous=True)
	print('Hello')
	darwin = Darwin()
	rospy.loginfo("Darwin initialization finished")
	print("Hello")
	
	rospy.Subscriber('tf', tf2_msgs.msg.TFMessage, callback, darwin)
	rospy.spin()



