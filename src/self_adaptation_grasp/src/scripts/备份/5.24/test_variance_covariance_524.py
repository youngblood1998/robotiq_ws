#!/usr/bin/env python
#coding=utf-8
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for receiving and interpreting the status of a 2F gripper.

This serves as an example for receiving messages from the 'Robotiq2FGripperRobotInput' topic using the 'Robotiq2FGripper_robot_input' msg type and interpreting the corresponding status of the 2F gripper.
"""
import std_msgs.msg
from takktile_ros.msg import Raw, Touch, Contact, Info
import rospy
import std_msgs as std_msgs
from std_srvs.srv import Empty
import os
import matplotlib.pyplot as plt
# import matplotlib.animation as animation
import time
import numpy as np
pre_data = [0]
test1 ='/home/wang/touth_classification/src/generate_dataset/slide_classify/test3'
i = 0
data_buffer4 = []
data_buffer9 = []
pre_data = [0,0,0,0,0]
current_data = [0,0,0,0,0]
def text_save(filename, data):  #filename为写入CSV文件的路径，data为要写入数据列表.
    file = open(filename, 'a')
    for i in range(len(data)):
        s = str(int(data[i])).replace('[','').replace(']','')#去除[],这两行按数据不同，可以选择
        s = s+' '   #去除单引号，逗号，每行末尾追加换行符
        file.write(s)
    file.write('\n')
    file.close()
#单个传感器方差
# def callback(data):
#     global i
#     global data_buffer4
#     global data_buffer9
#     global pub
#     global current_data
#     global pre_data
#     if i <20:
#         i+=1
#         a = np.array(data.pressure)
#         data_buffer4.append(a[4])
#     else:
#         current_data = data_buffer4
#         if pre_data == [0]*5:
#             pre_data = current_data
#         i+=1
#         covariance= np.cov(pre_data, current_data)
#         print(covariance)
#         array = [covariance[0][0],covariance[0][1],covariance[1][1]]
#         left_top = std_msgs.msg.Float64MultiArray(data=array)
#         pub.publish(left_top)
#         pre_data = current_data
#
#         # print(i,':',data_buffer)
#
#
#
#         i = 0
#         data_buffer4 = []
#两传感器协方差
def callback(data):
    global i
    global data_buffer4
    global data_buffer9
    global pub
    global current_data1
    global pre_data1
    global current_data2
    global pre_data2
    if i <3:
        i+=1
        data_buffer4.append(data.pressure[4])
        data_buffer9.append(data.pressure[3])
    else:
        current_data1 = data_buffer4
        current_data2 = data_buffer9
        print(current_data1,'\n', current_data2)
        i+=1
        covariance= np.cov((current_data1[0],current_data1[-1]),(current_data2[0],current_data2[-1]))
        # print(covariance)
        array = [covariance[0][0],covariance[0][1],covariance[1][1]]
        left_top = std_msgs.msg.Float64MultiArray(data=array)
        pub.publish(left_top)

        # print(i,':',data_buffer)



        i = 0
        data_buffer4 = []
        data_buffer9 = []

# 传感器总值方差
# def callback(data):
#     global i
#     global data_buffer4
#     global data_buffer9
#     global pub
#     global current_data
#     global pre_data
#     if i <5:
#         i+=1
#         a = np.array(data.pressure)
#         sum = np.sum(a)
#         data_buffer4.append(sum)
#         print(sum)
#         print(data_buffer4)
#     else:
#         current_data = data_buffer4
#         if pre_data == [0,0,0,0,0]:
#             pre_data = current_data
#         i+=1
#         covariance= np.cov(pre_data, current_data)
#         print(covariance)
#         array = [covariance[0][0],covariance[0][1],covariance[1][1]]
#         left_top = std_msgs.msg.Float64MultiArray(data=array)
#         pub.publish(left_top)
#         pre_data = current_data
#
#         # print(i,':',data_buffer)
#
#
#
#         i = 0
#         data_buffer4 = []


def listener():
    global pub


    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('cov',std_msgs.msg.Float64MultiArray,queue_size=1000)

    rospy.Subscriber('takktile/calibrated', Touch, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    listener()
