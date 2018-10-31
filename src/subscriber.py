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

shoulder_coord_l = (0.0, 0.0, 0.0)
hand_coord_l = (0.0, 0.0, 0.0)
elbow_coord_l = (0.0, 0.0, 0.0)
shoulder_coord_r = (0.0, 0.0, 0.0)
hand_coord_r = (0.0, 0.0, 0.0)
elbow_coord_r = (0.0, 0.0, 0.0)

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
	global hand_coord_l
	global shoulder_coord_l
	global elbow_coord_l
	global hand_coord_r
	global shoulder_coord_r
	global elbow_coord_r
	for tr in data.transforms:
		if tr.child_frame_id.startswith('left_hand'):
			translation = tr.transform.translation
			hand_coord_l = (translation.y, translation.x, translation.z)
		if tr.child_frame_id.startswith('left_shoulder'):
			translation = tr.transform.translation
			shoulder_coord_l = (translation.y, translation.x, translation.z)
		if tr.child_frame_id.startswith('left_elbow'):
			translation = tr.transform.translation
			elbow_coord_l = (translation.y, translation.x, translation.z)
		if tr.child_frame_id.startswith('right_hand'):
			translation = tr.transform.translation
			hand_coord_r = (translation.y, translation.x, translation.z)
		if tr.child_frame_id.startswith('right_shoulder'):
			translation = tr.transform.translation
			shoulder_coord_r = (translation.y, translation.x, translation.z)
		if tr.child_frame_id.startswith('right_elbow'):
			translation = tr.transform.translation
			elbow_coord_r = (translation.y, translation.x, translation.z)

	relative_elbow_l = sub(shoulder_coord_l, elbow_coord_l)
	relative_hand_l = sub(shoulder_coord_l, hand_coord_l)
	relative_elbow_r = sub(shoulder_coord_r, elbow_coord_r)
	relative_hand_r = sub(shoulder_coord_r, hand_coord_r)

	(phi_elbow, theta_elbow) = angle(relative_elbow_l)

	elbow_to_hand_l = sub(relative_elbow_l, relative_hand_l)
	angle_elbow_to_hand_l = angle_vectors(elbow_to_hand_l, relative_elbow_l)
	elbow_to_hand_r = sub(relative_elbow_r, relative_hand_r)
	angle_elbow_to_hand_r = angle_vectors(elbow_to_hand_r, relative_elbow_r)

	print('Rel.elbow ', relative_elbow_l)
	print('Rel.hand ', relative_hand_l)
	print('El 2 hand ', elbow_to_hand_l)
	print('Angle2hand ', angle_elbow_to_hand_l)
	print('Phi ', phi_elbow)
	print('Theta ', theta_elbow)

	elbow_to_axis_z_l = math.acos(relative_elbow_l[2] / math.sqrt(relative_elbow_l[0] ** 2 + relative_elbow_l[1] ** 2 + relative_elbow_l[2] ** 2)).real -0.25
	elbow_to_axis_y_l = math.acos(relative_elbow_l[0] / math.sqrt(relative_elbow_l[0] ** 2 + relative_elbow_l[1] ** 2 + relative_elbow_l[2] ** 2)).real
	if relative_elbow_l[2] < 0:
		elbow_to_axis_y_l = -elbow_to_axis_y_l

	elbow_to_axis_z_r = math.acos(relative_elbow_r[2] / math.sqrt(relative_elbow_r[0] ** 2 + relative_elbow_r[1] ** 2 + relative_elbow_r[2] ** 2)).real +0.25
	elbow_to_axis_y_r = math.acos(relative_elbow_r[0] / math.sqrt(relative_elbow_r[0] ** 2 + relative_elbow_r[1] ** 2 + relative_elbow_r[2] ** 2)).real
	if relative_elbow_r[2] < 0:
		elbow_to_axis_y_r = -elbow_to_axis_y_r

	print('ELBOW_TO_Z: ', elbow_to_axis_z_l)
	print('ELBOW_TO_Y: ', elbow_to_axis_y_l)

	shoulder_l = -(3.14 - elbow_to_axis_y_l)
	high_arm_l = 1.57 - elbow_to_axis_z_l
	shoulder_r = -(3.14 - elbow_to_axis_y_r)
	high_arm_r = 1.57 - elbow_to_axis_z_r
	print('Shoulder L: ', shoulder_l)
	print('HighArm L: ', high_arm_l)
	print('Shoulder R: ', shoulder_r)
	print('HighArm R: ', high_arm_r)
	print(' ')

	darwin.set_angles({"j_shoulder_l": shoulder_l})

	darwin.set_angles({"j_high_arm_l": high_arm_l})

	darwin.set_angles({"j_shoulder_r": shoulder_r})

	darwin.set_angles({"j_high_arm_r": high_arm_r})

	if angle_elbow_to_hand_l > 1.75:
		darwin.set_angles({"j_low_arm_l": -1.5})
	elif angle_elbow_to_hand_l < 0.25:
		darwin.set_angles({"j_low_arm_l": 0})
	else:
		darwin.set_angles({"j_low_arm_l": -(angle_elbow_to_hand_l - 0.25)})

	if angle_elbow_to_hand_r > 1.75:
		darwin.set_angles({"j_low_arm_r": -1.5})
	elif angle_elbow_to_hand_r < 0.25:
		darwin.set_angles({"j_low_arm_r": 0})
	else:
		darwin.set_angles({"j_low_arm_r": -(angle_elbow_to_hand_r - 0.25)})

	


if __name__ == '__main__':
	rospy.init_node("walker_demo", anonymous=True)
	print('Hello')
	darwin = Darwin()
	rospy.loginfo("Darwin initialization finished")
	print("Hello")
	
	rospy.Subscriber('tf', tf2_msgs.msg.TFMessage, callback, darwin)
	rospy.spin()



