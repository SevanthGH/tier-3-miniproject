#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
import time
import math
import ast
from tf.transformations import euler_from_quaternion



pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

angle_min= -1.57079637051
angle_max= 1.53938043118
angle_incr = 0.0314159281552
#fornt = [0]*10
goal_1= [0]*2
goal_2= [0]*2
goal_3= [0]*2
goal_1 = [3.5,9]
goal_2 = [7,-9.5]
goal_3 = [-6.5,-13.5]
Flag = True

x = y = angle = x1 = y1 = 0
count = Twist()                         # Create a var of type Int32
#pos = ModelState()


def position_robot(msg1):
    	global angle, x, y, z
    	x = msg1.pose[1].position.x
    	y = msg1.pose[1].position.y
    	quatern = ([msg1.pose[1].orientation.x, msg1.pose[1].orientation.y, msg1.pose[1].orientation.z, msg1.pose[1].orientation.w])
    	euler = euler_from_quaternion(quatern)
    	angle = euler[2]
        if angle <= 0:
            z = math.pi + (math.pi - abs(angle))
        else:
            z = angle
	return angle

def cal_angle_for_goals():
	global goals_1, x, y
	inc_x = goal_1[0] - x
	inc_y = goal_1[1] - y
	angle_goal1 = math.atan2(inc_y, inc_x)
	return angle_goal1

def cal_angle_for_goals2():
	global goals_2, x, y
	inc_x = goal_2[0] - x
	inc_y = goal_2[1] - y
	angle_goal2 = math.atan2(inc_y, inc_x)
        if angle_goal2 < 0:
           angle_goal2 = 2*math.pi - abs((angle_goal2))
	return math.degrees(angle_goal2)


def get_length(scan_data):
	return len(scan_data)

def get_index_of_closest_point(scan_data):
        m = min(i for i in scan_data if i > 0)
        index_min_dist = scan_data.index(m)
	return index_min_dist

def get_angle_of_closest_point(scan_data):
        angle_min_dist = angle_min+get_index_of_closest_point(scan_data)*angle_incr
	return angle_min_dist

def move_forward():
        count.linear.x = 0.1
        count.angular.z = 0.0

def turn_left():
        count.linear.x = 0.0
        count.angular.z = 0.1

def turn_right():
        count.linear.x = 0.0
        count.angular.z = -0.1

def stop():
    count.linear.x = 0.0
    count.angular.z = 0.0

def EU_distance(x,y,x1,y1):
    return math.sqrt((x1-x)**2 + (y1-y)**2)


def move_right_obst():
    for b in range(0,95):
    	turn_right()
        pub.publish(count)
        rate = rospy.Rate(5)
        rate.sleep()

def move_ahead():
    for a in range(0,20):
        count.linear.x = 0.1
        count.angular.z = 0.0
        rate = rospy.Rate(5)
        pub.publish(count)
        rate.sleep()

def move_left_obst():
    for b in range(0,60):
    	turn_left()
        pub.publish(count)
        rate.sleep()
    for a in range(0,10):
        count.linear.x = 0.1
        count.angular.z = 0.0
        pub.publish(count)
        rate.sleep()



def my_callback(msg):
	global goals_1, goal_2, goal_3, angle, Flag, scan_data, z, front, left, right, a

	scan_data = msg.ranges
    	front = msg.ranges[355:359] + msg.ranges[0:4]
    	left = msg.ranges[85:95]
    	right = msg.ranges[265:275]

	rate = rospy.Rate(5)

	if math.degrees(z) < math.degrees(cal_angle_for_goals()) and Flag == False:
           turn_left()
        if math.degrees(z) > math.degrees(cal_angle_for_goals()) and Flag == False:
           move_forward()
           if EU_distance(x,y,goal_1[0],goal_1[1]) <= 0.5:
              stop()
              Flag = True
              print 2, math.degrees(z), (cal_angle_for_goals2())
        if math.degrees(z) < cal_angle_for_goals2() and Flag == True:
           turn_left()
           print 3, math.degrees(z), (cal_angle_for_goals2())
        if math.degrees(z) >= (cal_angle_for_goals2()) and Flag == True:
           move_forward()
           if EU_distance(x,y,goal_2[0],goal_2[1]) <= 0.2:
              stop()
           #print front, left
           if min(front) < 0.5:
              print 5
              move_right_obst()
              move_ahead()








		#if abs(z) < cal_angle_for_goals2() and Flag == True:
		#	turn_left()
		#	print 3, z
		#if z > cal_angle_for_goals2() and Flag == True:
		#	move_forward()
            #if EU_distance(x,y,goal_2[0],goal_2[1]) <= 0.1:
			#	stop()
	#if min(msg.ranges[0:5]) < 0.5 or min(msg.ranges[355:359]) < 0.5:
	#	for b in range(0,25):
	#		turn_right()
    #    	pub.publish(count)
    #    	rate.sleep()


	pub.publish(count)


rospy.init_node('mini_prj')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
sub = rospy.Subscriber('/scan', LaserScan, my_callback)
model_state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, position_robot)
rospy.spin() # mantain the service open.
