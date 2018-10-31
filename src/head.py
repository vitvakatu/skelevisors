#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from os import system
import tf2_msgs.msg
import roslib
import cmath as math
import geometry_msgs.msg as msgs
import turtlesim.srv
from darwin_gazebo.darwin import Darwin

shoulder_coord = (0.0, 0.0, 0.0)
hand_coord = (0.0, 0.0, 0.0)
elbow_coord = (0.0, 0.0, 0.0)
shoulder_rotation = (0.0, 0.0, 0.0)
hand_rotation = (0.0, 0.0, 0.0)
elbow_rotation = (0.0, 0.0, 0.0)

def sub(s, f):
	return (f[0] - s[0], f[1] - s[1], f[2] - s[2])

def length(v):
	return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)

def angle(v):
	x = v[0] * 100
	y = v[1] * 100
	z = v[2] * 100
	phi = math.atan(z / math.sqrt(x ** 2 + y ** 2))
	theta = math.acos(x / math.sqrt(y ** 2 + x ** 2))
	return (phi.real, theta.real)

def angle_vectors(v1, v2):
	up = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
	down = math.sqrt(v1[0] ** 2 + v1[1] ** 2 + v1[2] ** 2)
	down = down * math.sqrt(v2[0] ** 2 + v2[1] ** 2 + v2[2] ** 2)
	return math.acos(up / down).real

def translate(value, leftMin, leftMax, rightMin, rightMax):
	# Figure out how 'wide' each range is
	leftSpan = leftMax - leftMin
	rightSpan = rightMax - rightMin

	# Convert the left range into a 0-1 range (float)
	valueScaled = float(value - leftMin) / float(leftSpan)

	# Convert the 0-1 range into a value in the right range.
	return rightMin + (valueScaled * rightSpan)


def callback(data):
	global hand_coord
	global shoulder_coord
	global elbow_coord
	global shoulder_rotation
	global hand_rotation
	global elbow_rotation
	for tr in data.transforms:
		if tr.child_frame_id.startswith('head'):
			print(tr.transform)

	


if __name__ == '__main__':
	rospy.init_node("walker_demo", anonymous=True)
	print('Hello')
	rospy.loginfo("Darwin initialization finished")
	print("Hello")
	
	rospy.Subscriber('tf', tf2_msgs.msg.TFMessage, callback)
	rospy.spin()



