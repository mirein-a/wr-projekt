#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg  import Pose
from sensor_msgs.msg import LaserScan
import math




end_point = [8,8]
temp_goal = end_point
current_pose = [0.5, 0.5, 0]

def get_distance(x_goal, y_goal, x, y):
    return abs(math.sqrt(((x_goal-x)**2)+((y_goal-y)**2)))

def get_goal_angle(x_goal, y_goal, x, y):
    return math.atan2(y_goal-y, x_goal-x)

distance_from_goal = get_distance(end_point[0], end_point[1], 0.5, 0.5)
state = 0
start_memorized = False
start_reached = False
start_point = None
temp_goal = None


def evaluate(x, y):
	global regions
	# global circum_state
	global state
	global start_point
	global start_memorized
	global start_reached
	d = 1.5

	if state == 0:
		if regions['front'] > d and regions['front_right'] > d and regions['front_left'] > d:
			state = 0
		else:
			state = 1
			start_point = [x, y]
			start_memorized = True
	elif state == 1 and start_reached(x,y):
		state = 2
	elif state == 2 and temp_reached(x,y):
		state = 0
		start_point = None
		start_memorized = False


	

def start_reached(x, y):
	if start_point[0] == x and start_point[1] == y:
		return True
	else:
		return False

def temp_reached(x, t):
	if temp_goal[0] == x and temp_goal[1] == y:
		return True
	else:
		return False


# sledzenie sciany

def circumnavigate():
	d = 1.5
	global current_pose

	if regions['front'] < d and regions['front_right'] > d and regions['front_left'] > d:
		turn_left()
		print("turn left")
	elif regions['front'] > d and regions['front_right'] < d and regions['front_left'] > d:
		#follow_the_wall() #!!!
		#go_to_point(current_pose[0],current_pose[1],current_pose[2])
		find_wall()
		print("find wall")
	elif regions['front'] > d and regions['front_right'] > d and regions['front_left'] < d:
		turn_left()
		print("turn left")
	elif regions['front'] < d and regions['front_right'] < d and regions['front_left'] > d:
		turn_left()
		print("turn left")
	elif regions['front'] > d and regions['front_right'] < d and regions['front_left'] < d:
		follow_the_wall()
		print("follow the wall")
	elif regions['front'] < d and regions['front_right'] > d and regions['front_left'] < d:
		turn_left()
		print("turn left")
	elif regions['front'] < d and regions['front_right'] < d and regions['front_left'] < d:
		turn_left()
		print("turn left")



# skladowe sledzenia sciany
def turn_left():
	global new_vel
	new_vel.angular.z = 0.3

def follow_the_wall():
	global new_vel
	new_vel.linear.x = 0.5

def find_wall():
	global new_vel
	new_vel.angular.z = -0.3

# nic sie nie dzieje

def go_to_point(x, y, theta):

	tolerance = 0.1
	k_linear = 0.2
	k_angular = 1.0

	global new_vel
	global end_point
	x_g, y_g = end_point

	# distance_from_goal = abs(math.sqrt(((x_g-x)**2)+((y_g-y)**2)))
 #    	angle = math.atan2(y_g-y, x_g-x)
 #    	new_vel.angular.z = -0.3

	distance = get_distance(x_g, y_g, x, y)
	new_vel.linear.x = k_linear*distance

	angle = get_goal_angle(x_g, y_g, x, y)
	new_vel.angular.z = k_angular*(angle-theta)

	# if (abs(angle-theta)<=tolerance):
	# 	new_vel.angular.z = 0.0
	# 	new_vel.linear.x = 0.15

	if (distance <= tolerance):
		new_vel.linear.x = 0
		new_vel.angular.z = 0


# funkcja wywolywana przy przyjsciu danych ze skanera laserowego
def scan_callback(scan):
	#print scan

	global regions

	regions = {
		'front': min(min(scan.ranges[0:35] + scan.ranges[324:359]), 10),
		'front_left': min(min(scan.ranges[36:107]), 10),
		'left': min(min(scan.ranges[108:179]), 10),
		'right': min(min(scan.ranges[180:251]), 10),
		'front_right': min(min(scan.ranges[252:323]), 10)
		}


# funkcja wywolywana przy przyjsciu danych o lokalizacji robota
def odom_callback(odom):
	global new_vel
	global state
	global regions
	global temp_goal
	global distance_from_goal
	global current_pose

	pose = Pose()
	pose.x = odom.pose.pose.position.x
	pose.y = odom.pose.pose.position.y

	pose.theta = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w])[2]
	
	current_pose = [pose.x, pose.y, pose.theta]
	print "Pozycja x: ",odom.pose.pose.position.x
	print "Pozycja y: ",odom.pose.pose.position.y
	print "Pozycja theta: ",pose.theta

	evaluate(pose.x, pose.y)

	# states of going to the end goal

	if (state == 0):
		# no obstacle detected
		go_to_point(pose.x, pose.y, pose.theta)
		print("velocity: ", new_vel.linear.x, new_vel.angular.z)
		print("state 0")

	elif (state == 1):
		print("state 1")
		# obstacle detected - go around
		current_distance = get_distance(end_point[0], end_point[1], pose.x, pose.y)

		if (distance_from_goal > current_distance):
			temp_goal = [pose.x, pose.y]
			distance_from_goal = current_distance
		
		circumnavigate()

	elif (state ==2):
		print("state 2")
		# after obstacle detected - go to closest point to the end goal
		circumnavigate()

 

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
