#!/usr/bin/env python
"""
Description: This code implementes a finite-time control barrier function
	     to drive the turtlebot from an initial condition to a desired goal region
Author:      Mohit Srinivasan
Date: 	     04/04/2020
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from cvxopt import matrix
from cvxopt.blas import dot
from cvxopt.solvers import qp, options
from cvxopt import matrix, sparse

def get_pose(msg):
	
	global x
	global y
	global phi

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	rot_q = msg.pose.pose.orientation
	(roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	phi = yaw

def go_to_region():
	
	h = -1

	while (h < 0):
	
		# Barrier function declarations
		gamma = 10.0	
		x_state = np.array([[x], [y]])
		P_region = np.array([[1/(0.4)**2, 0], [0, 1/(0.4)**2]])	
		C = np.array([[4.0], [2.0]])
		h = 1.0 - np.dot(np.dot(np.transpose(x_state - C), P_region), x_state - C)

		# Quadratic program declarations
		A = np.dot(2.0 * np.transpose(x_state - C), P_region)
		B = gamma * np.sign(h) 
	
		# Quadratic program solver
		H = matrix(np.array([[1.0, 0], [0, 1.0]]), tc='d')
		f = matrix(np.array([[0], [0]]), tc='d')
		u_si = qp(H, f, matrix(A), matrix(B))

		# Obtain unicycle velocities using Diffeomorphism technique
		l = 0.4
		R = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
		L = np.array([[1.0, 0], [0, 1.0/l]])
		u_turtlebot = np.dot(np.dot(np.transpose(R), L), np.array(u_si['x']))

		# Publish the velocities to Turtlebot
		vel_msg.linear.x = u_turtlebot[0] * np.cos(phi)
		vel_msg.linear.y = u_turtlebot[0] * np.sin(phi)
		vel_msg.linear.z = 0.0

		vel_msg.angular.x = 0.0	
		vel_msg.angular.y = 0.0
		vel_msg.angular.z = u_turtlebot[1]
	
		velocity_publisher.publish(vel_msg)
	
if __name__ == '__main__':
	
	x = 0.0
	y = 0.0
	phi = 0.0

	# ROS initializations
	rospy.init_node('go_to_region', anonymous=True)
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	odom_sub = rospy.Subscriber('/odom', Odometry, get_pose)
	vel_msg = Twist()

	try:
		go_to_region()
	except rospy.ROSInterruptException:
		pass
