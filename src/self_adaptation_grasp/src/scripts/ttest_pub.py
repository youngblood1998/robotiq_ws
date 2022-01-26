#!/usr/bin/env python
# -*- coding:utf-8 -*-
import roslib;

roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from std_msgs.msg import String
import std_msgs
from scipy.stats import ttest_rel
from takktile_ros.msg import Touch
import time
takktile_buffer_data = []

def buffer_callback(data):
    global takktile_buffer_data
    takktile_buffer_data = data.data

def stability_test_ttset():
    global takktile_buffer_data
    takktile_buffer_data_ini = rospy.wait_for_message('takktile/data_buffer5',std_msgs.msg.Float64MultiArray).data
    rospy.sleep(0.2)
    takktile_buffer_data_final = rospy.wait_for_message('takktile/data_buffer5',std_msgs.msg.Float64MultiArray).data
    takktile_buffer_data_ini = list(takktile_buffer_data_ini[:-1])
    takktile_buffer_data_final = list(takktile_buffer_data_final[:-1])
    pmin_value = 1
    # print(takktile_buffer_data_ini,'1111111',len(takktile_buffer_data_ini),"22222",takktile_buffer_data_final,len(takktile_buffer_data_final))
    for i in range(10):
        ini_data = []
        final_data = []
        for j in range(5):
            ini_data.append(takktile_buffer_data_ini[10*j+i])
            final_data.append(takktile_buffer_data_final[10*j+i])
            j+=1
        # print(ini_data,final_data)
        ttest,pval=ttest_rel(ini_data,final_data)
        print(ini_data,final_data,sum(final_data),pval)
        # print(pval)
        delta = sum(final_data) - sum(ini_data)
        if abs(delta)>30 and pval < pmin_value:
            pmin_value = pval

        i+=1
    print(pmin_value)
    if pmin_value<0.005:
        stability_test_ttset_result = 1
    else:
        stability_test_ttset_result = 0
    ttset_pub.publish(stability_test_ttset_result)

    # a = [i for i in range(100)]
    # print(len(a))
    # for i in range(10):
    #     ini_data = []
    #     for j in range(10):
    #         print('j=',j)
    #         ini_data.append(a[10*j+i])
    #         j+=1
    #     print(ini_data)
    #     i+=1


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    pub_buffer = rospy.Publisher('takktile/data_buffer5', std_msgs.msg.Float64MultiArray, queue_size=1000)

    rospy.Subscriber('takktile/data_buffer5', std_msgs.msg.Float64MultiArray, buffer_callback)
    ttset_pub = rospy.Publisher('ttest_result', std_msgs.msg.Float64, queue_size=1000)

    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        stability_test_ttset()

