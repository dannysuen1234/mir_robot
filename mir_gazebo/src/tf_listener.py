#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

path = []
global prev 
prev= time.time()

def callback(data):

		path.append(data)
		


def organize():
	for i in range (len(path)):
		tx = path[i].pose.pose.position.x
		ty = path[i].pose.pose.position.y
		tz = path[i].pose.pose.position.z

		rx = path[i].pose.pose.orientation.x
		ry = path[i].pose.pose.orientation.y
		rz = path[i].pose.pose.orientation.z
		rw = path[i].pose.pose.orientation.w
		path[i] = [tx, ty, tz, rx, ry, rz, rw]

def listener():
	rospy.init_node('tf_listener', anonymous=True)
	
	rospy.Subscriber("/odom", Odometry, callback)

	rospy.spin()

if __name__ == '__main__':

	listener()
	organize()
	with open('tf.txt', 'w') as f:
    		for item in path:
        		f.write("%s\n" %item)
