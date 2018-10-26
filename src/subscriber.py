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


def callback(data, darwin):
	global hand_coord
	global shoulder_coord
	global elbow_coord
	for tr in data.transforms:
		if tr.child_frame_id.startswith('left_hand'):
			translation = tr.transform.translation
			hand_coord = (translation.y, translation.x, translation.z)
		if tr.child_frame_id.startswith('left_shoulder'):
			translation = tr.transform.translation
			shoulder_coord = (translation.y, translation.x, translation.z)
		if tr.child_frame_id.startswith('left_elbow'):
			translation = tr.transform.translation
			elbow_coord = (translation.y, translation.x, translation.z)

	relative_elbow = sub(shoulder_coord, elbow_coord)
	relative_hand = sub(shoulder_coord, hand_coord)

	(phi_elbow, theta_elbow) = angle(relative_elbow)

	elbow_to_hand = sub(relative_elbow, relative_hand)
	angle_elbow_to_hand = angle_vectors(elbow_to_hand, relative_elbow)

	print('Rel.elbow ', relative_elbow)
	print('Rel.hand ', relative_hand)
	print('El 2 hand ', elbow_to_hand)
	print('Angle2hand ', angle_elbow_to_hand)
	print('Phi ', phi_elbow)
	print('Theta ', theta_elbow)

	elbow_to_z = math.asin(math.sqrt(elbow_to_hand[0] ** 2 + elbow_to_hand[1] ** 2) / math.sqrt(elbow_to_hand[0] ** 2 + elbow_to_hand[1] ** 2 + elbow_to_hand[2] ** 2)).real
	print('ELBOW_TO_Z: ', elbow_to_z)
	print(' ')

	if angle_elbow_to_hand > 1.75:
		darwin.set_angles({"j_low_arm_l": -1.5})
	elif angle_elbow_to_hand < 0.25:
		darwin.set_angles({"j_low_arm_l": 0})
	else:
		darwin.set_angles({"j_low_arm_l": -(angle_elbow_to_hand - 0.25)})
	
	if phi_elbow > 1.33:
		darwin.set_angles({"j_high_arm_l": -1.5})
	elif phi_elbow < -1.2:
		darwin.set_angles({"j_high_arm_l": 1.5})
	else:
		darwin.set_angles({"j_high_arm_l": -translate(phi_elbow, -1.2, 1.33, -1.5, 1.5)})
	
	if elbow_to_z > 1.75:
		darwin.set_angles({"j_shoulder_l": -1.5})
	elif elbow_to_z < 0.25:
		darwin.set_angles({"j_shoulder_l": 0})
	else:
		darwin.set_angles({"j_shoulder_l": -elbow_to_z})


if __name__ == '__main__':
	rospy.init_node("walker_demo", anonymous=True)
	print('Hello')
	darwin = Darwin()
	rospy.loginfo("Darwin initialization finished")
	print("Hello")
	
	rospy.Subscriber('tf', tf2_msgs.msg.TFMessage, callback, darwin)
	rospy.spin()



