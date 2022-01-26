#!/usr/bin/env python
# -*- coding:utf-8 -*-

import roslib;
roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from std_msgs.msg import String
import std_msgs
from takktile_ros.msg import Touch
import time
# import matplotlib.animation as animation
takktile_cur = []
takktile_buffer = []
takktile_buffer_final = []
takktile_counter = 0
BUFFER = 5

#   100hz传感器信息
def takktile_callback(data):
	global takktile_cur
	global BUFFER
	global takktile_counter
	global takktile_buffer_final
	global takktile_buffer
	takktile_cur = list(data.data)
	if takktile_counter < BUFFER:
		takktile_buffer_final = [0]
		takktile_counter += 1
		takktile_buffer.extend(takktile_cur)
	else:
		takktile_counter = 0
		takktile_buffer_final = takktile_buffer
		takktile_buffer = []
		time1 = time.time()
		takktile_buffer_final.append(time1)
		pub_msg = std_msgs.msg.Float64MultiArray(data = takktile_buffer_final)
		pub_buffer.publish(pub_msg)

if __name__ == '__main__':
	rospy.init_node('listener', anonymous=True)
	pub_buffer = rospy.Publisher('takktile/data_buffer5', std_msgs.msg.Float64MultiArray, queue_size=1000)
	rospy.Subscriber('/pressure', std_msgs.msg.Float64MultiArray, takktile_callback)
	while not rospy.is_shutdown():
		rospy.spin()
