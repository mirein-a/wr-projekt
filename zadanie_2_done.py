#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg  import Pose
from sensor_msgs.msg import LaserScan
import turtlesim
import math
from turtlesim.srv import SetPen
from std_srvs.srv import Empty as EmptyServiceCall

no1=[(2,4), (7,3), (5,7), (3,4)]
no2=[(1,3), (6,6), (10,7)]
no3=[(1,3), (7,2), (1,10), (5.54,5.54)]
no4=[(1,2), (10,7), (2,4)]

route = [(2,4), (7,3), (5,7), (3,4)]
index = 0

def get_distance(x_goal, y_goal, x, y):
    return abs(math.sqrt(((x_goal-x)**2)+((y_goal-y)**2)))

def get_goal_angle(x_goal, y_goal, x, y):
    return math.atan2(y_goal-y, x_goal-x)

# def spawn_new_turtle(x, y, theta):
#     global index
#     spawn_turtle = rospy.ServiceProxy('spawn_turtle', Spawn)
#     name = "turtle_" + str(index)
#     spawn_turtle(x, y, theta, name)
#     rospy.spin()


def turtlesim_pose_callback(data):
    turtle1_set_pen(0,0,0,1,0)
    global new_vel
    pose = turtlesim.msg.Pose()
    pose.x = round(data.x, 4)
    pose.y = round(data.y, 4)
    pose.theta = data.theta
    print "Pozycja x: ",pose.x
    print "Pozycja y: ",pose.y
    print "Pozycja theta: ",pose.theta

    global route
    global index
    k_linear = 0.4
    k_angular = 2.0
    tolerance = 0.4

    x_g, y_g = route[index]
    print "next: ", x_g, y_g
    distance = get_distance(x_g, y_g, pose.x, pose.y)
    vel_linear = k_linear*distance

    angle = get_goal_angle(x_g, y_g, pose.x, pose.y)
    vel_angular = k_angular*(angle-pose.theta)

    print "distance: ", distance

    new_vel.linear.x = vel_linear
    new_vel.angular.z = vel_angular

    if (distance<tolerance):
        print "distance less than tolerance"
        if (index<(len(route)-1)):
            index += 1
            x_g = route[index][0]
            y_g = route[index][1]

            turtle1_set_pen(255,0,0,5,0)
            # spawn_new_turtle(pose.x, pose.y, pose.theta)
        else:
            new_vel.linear.x = 0
            new_vel.angular.z = 0

if __name__== "__main__":
	global new_vel
	new_vel = Twist()
	rospy.init_node('wr_zad', anonymous=True)
	print("ready")
    
        rospy.wait_for_service('reset')
        reset_sim = rospy.ServiceProxy('reset', EmptyServiceCall)
        reset_sim()

        rospy.wait_for_service('turtle1/set_pen')
        turtle1_set_pen = rospy.ServiceProxy('turtle1/set_pen', SetPen)

        rospy.Subscriber( '/turtle1/pose' , turtlesim.msg.Pose, turtlesim_pose_callback)
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

	rate=rospy.Rate(10) # 10Hz
	while not rospy.is_shutdown():
		pub.publish(new_vel) #wyslanie predkosci zadanej
		rate.sleep()

	print("END")

