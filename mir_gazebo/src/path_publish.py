import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math

global odom
odom = []

rospy.init_node("mynode")

goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)



def listener():
	odom = rospy.wait_for_message("amcl_pose", PoseWithCovarianceStamped)
	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y
	return [x, y]


with open("tf.txt", "r") as f:
	a = f.read()
	b = a.split("\n")
	b.pop()
	for i in range (len(b)):
		b[i] = b[i].strip("[")
		b[i] = b[i].strip("]")

	for i in range (len(b)):
		b[i] = b[i].split(",")

	for i in range (len(b)):
		for j in range (len(b[i])):
			b[i][j] = float((b[i][j]))

def distance(x1, y1, x2, y2):
	return math.sqrt((x1-x2)**2 + (y1-y2)**2)

for item in b:
	goal = PoseStamped()
	goal.header.seq = 1
	goal.header.stamp = rospy.Time.now()
	goal.header.frame_id = "map"
	goal.pose.position.x = item[0]
	goal.pose.position.y = item[1]
	goal.pose.position.z =item[2]

	goal.pose.orientation.x = item[3]
	goal.pose.orientation.y = item[4]
	goal.pose.orientation.z = item[5]
	goal.pose.orientation.w = item[6]
	rospy.sleep(0.5)
	goal_publisher.publish(goal)
	print(goal)
	

	while True:
		curr_x, curr_y = listener()
		d = distance(item[0], item[1], curr_x, curr_y)
		print(d, curr_x, curr_y, item[0], item[1])
		
		if d <1.5 :
			break




