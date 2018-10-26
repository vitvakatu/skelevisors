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
elbow_coord = (0.0, 0.0, 0.0)

def sub(s, f):
	return (f[0] - s[0], f[1] - s[1], f[2] - s[2])

def length(v):
	return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)

def angle(v):
	x = v[0] * 100
	y = v[1] * 100
	z = v[2] * 100
	phi = math.atan(x / math.sqrt(z ** 2 + y ** 2))
	theta = math.acos(z / math.sqrt(y ** 2 + z ** 2))
	return (phi, theta)


def callback(data):
	global shoulder_coord
	global elbow_coord
	for tr in data.transforms:
		if tr.child_frame_id.startswith('left_elbow'):
			translation = tr.transform.translation
			elbow_coord = (translation.x, translation.y, translation.z)
		if tr.child_frame_id.startswith('left_shoulder'):
			translation = tr.transform.translation
			shoulder_coord = (translation.x, translation.y, translation.z)

	print(shoulder_coord)
	print(elbow_coord)
	
	print(sub(shoulder_coord, elbow_coord))
	print('Length: ', length(sub(shoulder_coord, elbow_coord)))
	
	print('(Phi, Theta): ', angle(sub(shoulder_coord, elbow_coord)))

if __name__ == '__main__':
	rospy.init_node("walker_demo", anonymous=True)
	print('Hello')
	#darwin = Darwin()
	rospy.loginfo("Darwin initialization finished")
	print("Hello")
	
	rospy.Subscriber('tf', tf2_msgs.msg.TFMessage, callback)
	rospy.spin()



