#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg  import Pose
from sensor_msgs.msg import LaserScan
import math


# NOWE TYLKO DLA ALG1:

def on_m_line(x,y):
	global end_point
	global init_point
	tolerance = 0.1

	a = (end_point[1] - init_point[1])/(end_point[0] - init_point[0])
	b = end_point[1] - a*end_point[0]

	if (abs(y - a*x - b) <= tolerance):
		print("ON M LINE")
		return True
	else:
		return False


# STARE Z BUG1:

init_point = [0.5,0.5]
end_point = [8,8]
no_wall = True
far_away = True
d = 1.5


def get_distance(x_goal, y_goal, x, y):
    return abs(math.sqrt(((x_goal-x)**2)+((y_goal-y)**2)))

def get_goal_angle(x_goal, y_goal, x, y):
    return math.atan2(y_goal-y, x_goal-x)

state = 0
start_point = None

def evaluate(x, y):
	global regions
	# global circum_state
	global state
	global start_point
	global start_memorized
	global start_reached
	global d

	if state == 0:
		d = 1.5
		if no_wall:
			state = 0
		else:
			state = 1
			start_point = [x, y]
	elif state == 1:
		if on_m_line(x,y):
			state = 3
		elif start_reached(x, y):
			state = 2
	elif state == 2:
		if on_m_line(x,y):
			start_point = None
			state = 3
	elif state == 3:
		if no_wall:
			state = 3
			d = 1.0
		elif far_away:
			state = 0
		else:
			state = 1
			start_point = [x, y]


	

def start_reached(x, y):
	if start_point[0] == x and start_point[1] == y:
		return True
	else:
		return False


#  MODIFIED:

def circumnavigate():
	global d

	if regions['front'] < d and regions['front_right'] > d and regions['front_left'] > d:
		if (state == 1):
			turn_left()
			print("turn left")
		else:
			turn_right()
			print("turn right")
	elif regions['front'] > d and regions['front_right'] < d and regions['front_left'] > d:
		if (state == 1):
			follow_the_wall()
			print("follow the wall")
		else:
			turn_right()
			print("turn right")
	elif regions['front'] > d and regions['front_right'] > d and regions['front_left'] < d:
		if (state == 1):
			turn_right()
			print("turn right")
		else:
			follow_the_wall()
			print("follow the wall")
	elif regions['front'] < d and regions['front_right'] < d and regions['front_left'] > d:
		if (state == 1):
			turn_left()
			print("turn left")
		else:
			turn_right()
			print("turn right")
	elif regions['front'] > d and regions['front_right'] < d and regions['front_left'] < d:
		if (state == 1):
			follow_the_wall()
			print("follow the wall")
		else:
			follow_the_wall()
			print("follow the wall")
	elif regions['front'] < d and regions['front_right'] > d and regions['front_left'] < d:
		if (state == 1):
			turn_left()
			print("turn left")
		else:
			turn_right()
			print("turn right")
	elif regions['front'] < d and regions['front_right'] < d and regions['front_left'] < d:
		if (state == 1):
				turn_left()
				print("turn left")
		else:
				turn_right()
				print("turn right")
	# elif regions['front'] > d and regions['front_right'] > d and regions['front_left'] > d:
	# 	turn_right()
	# 	print("turn right")
	elif regions['right'] > d and state == 1:
		turn_right()
		print("turn right")
	elif regions['left'] > d and state == 2:
		turn_left()
		print("turn left")







# skladowe sledzenia sciany
def turn_left():
	global new_vel
	new_vel.angular.z = 0.3

def turn_right():
	global new_vel
	new_vel.angular.z = -0.3

def follow_the_wall():
	global new_vel
	new_vel.linear.x = 0.15
	new_vel.angular.z = 0.0

# def find_wall():
# 	global new_vel
# 	new_vel.angular.z = -0.3

# nic sie nie dzieje

def go_to_point(x, y, theta):

	tolerance = 0.1

	global new_vel
	global end_point
	x_g, y_g = end_point

	angle = get_goal_angle(x_g, y_g, x, y)
	distance = get_distance(x_g, y_g, x, y)

	if (abs(angle-theta)<=tolerance):
		new_vel.angular.z = 0.0
	else:
		new_vel.angular.z = 0.3

	new_vel.linear.x = 0.15

	if (distance <= tolerance):
		new_vel.linear.x = 0
		new_vel.angular.z = 0



# funkcja wywolywana przy przyjsciu danych ze skanera laserowego
# MODOFIED:
def scan_callback(scan):
	#print scan

	global regions
	global no_wall
	global far_away

	regions = {
		'front': min(min(scan.ranges[0:35]), min(scan.ranges[324:359]), 10),
		'front_left': min(min(scan.ranges[36:107]), 10),
		'left': min(min(scan.ranges[108:179]), 10),
		'right': min(min(scan.ranges[180:251]), 10),
		'front_right': min(min(scan.ranges[252:323]), 10)
		}

	if regions['front'] > 1 and regions['front_right'] > 1 and regions['front_left'] > 1:
		no_wall = True
	else:
		no_wall = False

	if regions['right'] > 2 and regions['left'] > 2:
		far_away = True
	else:
		far_away = False


# funkcja wywolywana przy przyjsciu danych o lokalizacji robota
def odom_callback(odom):
	global new_vel
	global state
	global regions
	global d

	pose = Pose()
	pose.x = odom.pose.pose.position.x
	pose.y = odom.pose.pose.position.y

	pose.theta = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w])[2]
	
	# print "Pozycja x: ",odom.pose.pose.position.x
	# print "Pozycja y: ",odom.pose.pose.position.y
	# print "Pozycja theta: ",pose.theta
	print "d: ", d

	evaluate(pose.x, pose.y)

	# states of going to the end goal

	if (state == 0):
		# no obstacle detected
		go_to_point(pose.x, pose.y, pose.theta)
		print("velocity: ", new_vel.linear.x, new_vel.angular.z)
		print("state 0")

	elif (state == 1):
		print("state 1")
		print("velocity: ", new_vel.linear.x, new_vel.angular.z)
		# obstacle detected - go around
		current_distance = get_distance(end_point[0], end_point[1], pose.x, pose.y)
		
		circumnavigate()

	elif (state ==2):
		print("state 2")
		print("velocity: ", new_vel.linear.x, new_vel.angular.z)
		# after start point reached - circumnav in opposite direction
		circumnavigate()

	elif (state == 3):
		# no obstacle detected
		go_to_point(pose.x, pose.y, pose.theta)
		print("velocity: ", new_vel.linear.x, new_vel.angular.z)
		print("state 3")

 

if __name__== "__main__":
	global new_vel
	new_vel = Twist()
	rospy.init_node('wr_zad', anonymous=True)
	print("ready")
	rospy.Subscriber( '/odom' , Odometry, odom_callback)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber( '/scan' , LaserScan, scan_callback)
	
	rate=rospy.Rate(10) # 10Hz
	while not rospy.is_shutdown():
		pub.publish(new_vel)#wyslanie predkosci zadanej
		rate.sleep()

	print("END")
