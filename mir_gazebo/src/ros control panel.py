import Tkinter as tk
import os 
import subprocess
import time
import threading
import tf
import rospy
from tf2_msgs.msg import TFMessage

#define all variable here to use this panel in different ros packages
gazebo_package = 'husky_gazebo '
gazebo_launch = 'husky_combined.launch'
navigation_package = 'husky_navigation '
navigation_launch = 'amcl_demo.launch'
move_base_package = 'husky_gazebo '
move_base_launch = 'husky_move_base.launch'
rviz_package = 'husky_viz ' 
rviz_launch = 'nav.launch'
map_path_pmg = "/home/vtl/catkin_ws/src/husky/husky_navigation/maps/map_before_edit.pgm"
map_path_yaml = '/home/vtl/catkin_ws/src/husky/husky_navigation/maps/map_before_edit.yaml'
good_map_path_yaml = '/home/vtl/catkin_ws/src/husky/husky_navigation/maps/map_after_edit.yaml'
save_map_path = '/home/vtl/catkin_ws/src/husky/husky_navigation/maps/map_before_edit'

def listener():

	rospy.init_node('listener', anonymous=True)
	a =rospy.wait_for_message("tf", TFMessage)
	return(a)

win = tk.Tk()
win.title("SLAM tutorial")

position_cmd = '''rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped  "header: 
  seq: 
  stamp: now
  frame_id: "map"
pose: 
  pose: 
    position: 
      x: 0.731374
      y: -0.044159
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.0395097685597
      w: 0.999219184258
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]"'''


point_1 = '''rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header: 
  seq: 
  stamp: now
  frame_id: "map"
pose: 
  position: 
    x: -16.8
    y: -22.5
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"'''

point_2 = '''rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header: 
  seq: 
  stamp: now
  frame_id: "map"
pose: 
  position: 
    x: -5.6
    y: -0.7
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"'''

point_3 = '''rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header: 
  seq: 
  stamp: now
  frame_id: "map"
pose: 
  position: 
    x: 20
    y: -0.85
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"'''

point_4 = '''rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header: 
  seq: 
  stamp: now
  frame_id: "map"
pose: 
  position: 
    x: 17.7
    y: -11.3
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"'''

point_5 = '''rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header: 
  seq: 
  stamp: now
  frame_id: "map"
pose: 
  position: 
    x: -10
    y: 13.5
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"'''


#functions 
def linux_command(*command):
	cmd = ''.join(command)
	os.system(cmd)

def test_pos(event):
	os.system(position_cmd)




#def navigation_pose():
#	os.system(position_cmd)

#call back functions

def stop_record(event):

	os.system("pkill -f tf_listener.py")

def record_path(event):
	record_path_thread = threading.Thread(target = record_path_cmd, args=())
	record_path_thread.start()

def use_path(event):
	use_path_thread = threading.Thread(target = use_path_cmd, args=())
	use_path_thread.start()

def move_base(event):
	cmd = 'roslaunch ' +move_base_package + move_base_launch
	move_base_demo = threading.Thread(target = open_move_base, args=())
	move_base_demo.start()

def btn_click (event):
	cmd = 'roslaunch ' + gazebo_package + ' ' + gazebo_launch
	y = threading.Thread(target = linux_command, args=cmd)
	y.start()

def btn_stop(event):
	cmd = "pkill roslaunch"
	x = threading.Thread(target = linux_command, args=cmd)
	x.start()

def edit_map_btn(event):
	cmd = 'gimp ' + map_path_pmg
	z = threading.Thread(target = linux_command, args = cmd)
	z.start()

def save_map(event):
	cmd = 'rosrun map_server map_saver -f ' + save_map_path
	a = threading.Thread(target = linux_command, args=cmd)
	a.start()



def navigation(event):
	cmd = 'roslaunch ' + navigation_package + navigation_launch 
	nav = threading.Thread(target = linux_command, args= cmd)
	nav.start()
	os.system(position_cmd)

	
def navigate_btn_1(event):

	n1 = threading.Thread(target = linux_command, args = point_1)
	n1.start()


def navigate_btn_2(event):
	
	n2 = threading.Thread(target = linux_command, args = point_2)
	n2.start()

def navigate_btn_3(event):
	
	n3 = threading.Thread(target = linux_command, args = point_3)
	n3.start()

def navigate_btn_4(event):
	
	n4 = threading.Thread(target = linux_command, args = point_4)
	n4.start()

def navigate_btn_5(event):
	
	n5 = threading.Thread(target = linux_command, args = point_5)
	n5.start()

def good_map_btn(event):
	cmd = 'rosrun map_server map_server ' + good_map_path_yaml
	gd_map = threading.Thread(target = linux_command, args = cmd)
	gd_map.start()

def bad_map_btn(event):
	cmd = 'rosrun map_server map_server ' + map_path_yaml
	bad_map = threading.Thread(target = bad_map_pub, args = ())
	bad_map.start()

def rviz(event):
	cmd = 'roslaunch ' + rviz_package + rviz_launch
	rviz_open = threading.Thread(target = linux_command, args=cmd)
	rviz_open.start()

def record_path_cmd():
	os.system("python tf_listener.py")

def use_path_cmd():
	os.system("python path_publish.py")

frame_a = tk.Frame()
frame_b = tk.Frame()

greeting = tk.Label(text="Control panel")
greeting.pack(fill = tk.BOTH, expand = True)

button1 = tk.Button(text = "Start Task 1 SLAM:", width =25, height =5, bg = "#618685", fg = "yellow")
button1.bind("<Button-1>", btn_click)
button1.pack(fill = tk.BOTH, expand = True)

button3 = tk.Button(master = frame_b, text = "Save Map", width =25, height =5, bg = "#80ced6", fg = "yellow")
button3.bind("<Button-1>", save_map)
button3.pack(fill = tk.BOTH, expand = True, side = tk.LEFT)

button4 = tk.Button(master = frame_b, text = "Edit Map", width =25, height =5, bg = "#80ced6", fg = "yellow")
button4.bind("<Button-1>", edit_map_btn)
button4.pack(fill = tk.BOTH, expand = True)

frame_b.pack()

button8 = tk.Button( text = "Start Task2 navigation", width =25, height =5, bg = "#618685", fg = "yellow")
button8.bind("<Button-1>", navigation)
button8.pack(fill = tk.BOTH, expand = True)

#test = tk.Button(text = "Send initial pose", width =25, height =5, bg = "#80ced6", fg = "yellow")
#test.bind("<Button-1>", test_pos)
#test.pack(fill = tk.BOTH, expand = True)


button5 = tk.Button(master = frame_a, text = "Point 1", width =8, height =5, bg = "#80ced6", fg = "yellow")
button5.bind("<Button-1>", navigate_btn_1)
button5.pack(fill=tk.BOTH, side = tk.LEFT)

button6 = tk.Button(master = frame_a, text = "Point 2", width =8, height =5, bg = "#80ced6", fg = "yellow")
button6.bind("<Button-1>", navigate_btn_2)
button6.pack(fill=tk.BOTH, side = tk.LEFT)

button7 = tk.Button(master = frame_a, text = "Point 3", width =8, height =5, bg = "#80ced6", fg = "yellow")
button7.bind("<Button-1>", navigate_btn_3)
button7.pack(fill=tk.BOTH, side = tk.LEFT)


btn_p4 = tk.Button(master = frame_a, text = "Point 4", width =8, height =5, bg = "#80ced6", fg = "yellow")
btn_p4.bind("<Button-1>", navigate_btn_4)
btn_p4.pack(fill=tk.BOTH, side = tk.LEFT)

btn_p5 = tk.Button(master = frame_a, text = "Point 5", width =8, height =5, bg = "#80ced6", fg = "yellow")
btn_p5.bind("<Button-1>", navigate_btn_5)
btn_p5.pack(fill=tk.BOTH, side = tk.LEFT)

frame_a.pack()

frame_d = tk.Frame()
good_map = tk.Button(master = frame_d, text = "Use good map", width =25, height =5, bg = "#80ced6", fg = "yellow")
good_map.bind("<Button-1>", good_map_btn)
good_map.pack(fill=tk.BOTH, side = tk.LEFT)

bad_map = tk.Button(master = frame_d, text = "Use bad map", width =25, height =5, bg = "#80ced6", fg = "yellow")
bad_map.bind("<Button-1>", bad_map_btn)
bad_map.pack(fill=tk.BOTH, side = tk.LEFT)

frame_d.pack()

frame_e = tk.Frame()

move_base_btn = tk.Button(master = frame_e, text = "move base demo", width =25, height =5, bg = "#618685", fg = "yellow")
move_base_btn.bind("<Button-1>", move_base)
move_base_btn.pack(fill = tk.BOTH, side = tk.LEFT)

rviz_btn = tk.Button(master = frame_e, text = "Open RVIZ", width =25, height =5, bg = "#618685", fg = "yellow")
rviz_btn.bind("<Button-1>", rviz)
rviz_btn.pack(fill = tk.BOTH, expand = True)

frame_e.pack()

frame_record = tk.Frame()
record_btn = tk.Button(master = frame_record, text = "record path 1", width =15, height =5, bg = "#618685", fg = "yellow")
record_btn.bind("<Button-1>", record_path)

record_btn.pack(fill = tk.BOTH, side = tk.LEFT)

path_btn = tk.Button(master = frame_record, text = "use path 1", width =15, height =5, bg = "#618685", fg = "yellow")
path_btn.bind("<Button-1>", use_path)
path_btn.pack(fill = tk.BOTH, side = tk.LEFT)

stop_btn = tk.Button(master = frame_record, text = "stop recording", width =15, height =5, bg = "#618685", fg = "yellow")
stop_btn.bind("<Button-1>", stop_record)
stop_btn.pack(fill = tk.BOTH, expand = True)
frame_record.pack()

button2 = tk.Button(text = "quit", width =25, height =5, bg = "red", fg = "yellow")
button2.bind("<Button-1>", btn_stop)
button2.pack(fill = tk.BOTH, expand = True)


win.mainloop()
