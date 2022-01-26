#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import math

x = 0
y = 0
theta = 0
rotate = True

def newOdom(msg):
    global x
    global y
    global theta
  

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def position(x1, y1, x2, y2):
	return ((x1-x2)**2 +(y1-y2)**2)**0.5

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

goal_list = [[19, 0], [0, 9], [-10, 0]]
curr_point = goal_list.pop(0)
goal = Point()
goal.x = curr_point[0]
goal.y = curr_point[1]

while not rospy.is_shutdown():

    inc_x = goal.x -x
    inc_y = goal.y -y
 
    angle_to_goal = atan2(inc_y, inc_x)
    

    if theta <0:
	theta += 3.14*2

    if angle_to_goal <0:
	angle_to_goal += 3.14*2

    if theta>6.18:
	theta -=6.28
    if angle_to_goal>6.18:
	angle_to_goal -=6.28



    if position(goal.x, goal.y, x, y) <0.1:
		print("arrived")
		if goal_list:
			curr_point = goal_list.pop(0)
			goal.x = x + curr_point[0]
			goal.y = y + curr_point[1]
			
		else:
			break
    elif abs(angle_to_goal-theta) > 0.1 :

        	speed.linear.x = 0.0
        	speed.angular.z = 0.3
	
	
    else:
	
	
	        speed.linear.x = 0.5
       		speed.angular.z = 0.0
		

    pub.publish(speed)

    r.sleep()
