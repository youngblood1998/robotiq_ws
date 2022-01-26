#!/usr/bin/env python
# -*- coding:utf-8 -*-

# import roslib;
# roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from std_msgs.msg import String
import std_msgs
from takktile_ros.msg import Touch

#   100hz传感器信息
def takktile_callback(data):
    p = list(data.pressure)
    if len(p) == 12:
        pressure = [-0.0054*p[0],
                    -0.0078*p[1],
                    -0.0050*p[2],
                    -0.0054*p[3],
                    -0.0044*p[4],
                    -0.0055*p[5],
                    -0.0051*p[6],
                    -0.0060*p[7],
                    -0.0041*p[8],
                    -0.0046*p[9],
                    -0.0044*p[10],
                    -0.0046*p[11]]
    else:
        pressure = [-0.0054*p[0],
                    -0.0078*p[1],
                    -0.0050*p[2],
                    -0.0054*p[3],
                    -0.0044*p[4],
                    -0.0022*p[4]-0.0027*p[0],
                    -0.0055*p[5],
                    -0.0051*p[6],
                    -0.0060*p[7],
                    -0.0041*p[8],
                    -0.0046*p[9],
                    -0.0044*p[10]]
    pressure = [round(i,4) for i in pressure]
    pub_msg = std_msgs.msg.Float64MultiArray(data = pressure)
    pub_buffer.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node('transformer', anonymous=True)
    pub_buffer = rospy.Publisher('pressure', std_msgs.msg.Float64MultiArray, queue_size=1000)
    rospy.Subscriber('takktile/calibrated', Touch, takktile_callback)
    print "transforming!"
    while not rospy.is_shutdown():
        rospy.spin()
