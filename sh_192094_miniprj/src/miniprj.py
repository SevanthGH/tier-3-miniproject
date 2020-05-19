#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
import time
import math
import ast
from tf.transformations import euler_from_quaternion
import numpy
from goal_publisher.msg import PointArray



pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

goal_1= [0]*2
goal_2= [0]*2
goal_3= [0]*2
goal_1 = [3.5,9]
goal_2 = [7,-9.5]
goal_3 = [-6.5,-13.5]
flag = False
theta = 0
sign = 1
x = y = angle = x1 = y1 = i = 0
i = 1
count = Twist()                         # Create a var of type Int32
#pos = ModelState()
goalx = [0.0]*20
goaly = [0.0]*20
goal = [0.0]*20
inc_x = [0.0]*20
inc_y = [0.0]*20
#ax = numpy.array([1.5, 2.5, 1.5, -3.5, -3.25, 3, -0.5, 0, -3.25, 2, -3.25, -3, 3, 0, -4.5, -11, -3, -3.25,-2.75, -1])
#by = numpy.array([0, 2.5, 4.5, 2, -2.75, -7, 6.5, 2.5, -0.25, 12, 7, 3, -1, -1.5, -1.75, 2.5, -4, 10, -9, -0.5])
#cx = numpy.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
'''                  [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])
'''
#print ax.dot(cx)
#print by.dot(cx)
#goalx = ax.dot(cx)
#goaly = by.dot(cx)
#goalx = [2.5, 1.5, 2, 0, 1.5, -3.5, -0.5, -3.25, -11, -3.25]
#goaly = [2.5, 4.5, 12, 2.5, 0, 2, 6.5, 7, 2.5, 10]
#goalx = [0, 3, 3, 2.5, 1.5, 2, -3.25, -3.25, -11, -4.5, -2.75, -3, -3.25, -3.25, -3.5, -3, -1, -0.5, 0, 1.5]
#goaly = [-1.5, -7, -1, 2.5, 4.5, 12, 10, 7, 2.5, -1.75, -9, -4, -2.75, -0.25, 2, 3, -0.5, 6.5, 2.5, 0]
#goalx = [1.5, 2.5, 1.5, -3.5, -3.25, 3, -0.5, 0, -3.25, 2, -3.25, -3, 3, 0, -4.5, -11, -3, -3.25,-2.75, -1]
#goaly = [0, 2.5, 4.5, 2, -2.75, -7, 6.5, 2.5, -0.25, 12, 7, 3, -1, -1.5, -1.75, 2.5, -4, 10, -9, -0.5]
#rospy.sleep(3)
def goal_points(msg2):
    global goalx, goaly, goal
    for a in range (len(msg2.goals)):
        #goalx = msg2.goals[a].x
        #goaly = msg2.goals[a].y
        goal = msg2.goals


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
	return z

def cal_angle_for_goals(i):
        #global i
	#global goalx, goaly, x, y, angle_goal, flag, sign
        global sign, inc_x, inc_y, angle_goal
	inc_x[i] = goal[i].x - x
	inc_y[i] = goal[i].y - y
	angle_goal = math.atan2(inc_y[i], inc_x[i])
    	if angle_goal < 0:
       	   angle_goal = 2*math.pi - abs(angle_goal)
        if math.degrees(angle_goal) - math.degrees(z) < 0:
       	   sign = -1
        #if abs(math.degrees(angle_goal) - math.degrees(z)) > 180:
        #   sign = sign*(-1)
        else:
           sign = 1
    	return math.degrees(angle_goal)

def move_forward():
        count.linear.x = 0.1
        count.angular.z = 0.0

def turn_left():
        count.linear.x = 0.0
        count.angular.z = 0.1*sign

def turn_right():
        count.linear.x = 0.0
        count.angular.z = -0.1

def stop():
    count.linear.x = 0.0
    count.angular.z = 0.0

def EU_distance(x,y,x1,y1):
    return math.sqrt((x1-x)**2 + (y1-y)**2)

def my_callback(msg):
	global goalx, goaly, angle, scan_data, z, front, left, right, i
        if i < 20:
           move_robot(msg)
        if i >= 20:
           stop()
           print 'completed'

	rate = rospy.Rate(100)
	pub.publish(count)


def move_robot(msg):
    global z, i, flag
    if abs(math.degrees(z) - cal_angle_for_goals(i)) > 5:
       #print math.degrees(z), cal_angle_for_goals(i)
       turn_left()
    if abs(math.degrees(z) - cal_angle_for_goals(i)) < 5:
       move_forward()
       obstacle(msg)
       #print EU_distance(x,y,goalx[i],goaly[i])
    if EU_distance(x,y,goal[i].x,goal[i].y) < 0.1:
       stop()
       print 'Reached goal', i, x, y#, '\n'"-"*100
       print '_________________________________________________________________'
       i = i + 1
       print 'Reaching next goal', goal[i].x,goal[i].y#, '\n'"-"*100
       print '-----------------------------------------------------------------'

       sign = 1


def move_right_obst():
    for b in range(0,65):
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

def obstacle(msg):
    scan_data = msg.ranges
    front = msg.ranges[340:359] + msg.ranges[0:25]
    left = msg.ranges[85:95]
    right = msg.ranges[265:275]
    if min(front) < 0.5:
       move_right_obst()
       move_ahead()






rospy.init_node('mini_prj')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
model_state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, position_robot)
goals = rospy.Subscriber('/goals', PointArray, goal_points)
rospy.sleep(3)
sub = rospy.Subscriber('/scan', LaserScan, my_callback)
rospy.spin() # mantain the service open.
