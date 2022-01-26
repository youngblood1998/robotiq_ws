#!/usr/bin/env python
#coding=utf-8
import roslib;

roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from std_msgs.msg import String
import std_msgs
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from takktile_ros.msg import Raw, Touch, Contact, Info
from std_srvs.srv import Empty
import os
import matplotlib.pyplot as plt
# import matplotlib.animation as animation
import time
import numpy as np
from scipy.stats import ttest_rel
takktile_buffer_data = []
takktile_counter = 0
BUFFER = 5
# cdict = {
#   'red'  :  ( (0.0, 0.25, .25), (0.02, .59, .59), (1., 1., 1.)),
#   'green':  ( (0.0, 0.0, 0.0), (0.02, .45, .45), (1., .97, .97)),
#   'blue' :  ( (0.0, 1.0, 1.0), (0.02, .75, .75), (1., 0.45, 0.45))
# }
#
# cm = m.colors.LinearSegmentedColormap('my_colormap', cdict, 1024)
#
#
#
#
# fig = plt.figure(1)
# # draw
# plt.imshow(data, interpolation='nearest', cmap=cm, origin='lower',vmin=-50, vmax=400)
#
# plt.xticks(())
# plt.yticks(())
# fig.canvas.update
full_data_filepath = '/home/wang/touth_classification/src/generate_dataset/dataset/79.4HA/79_4HA_fulldata_test_new'
k_filepath = '/home/wang/touth_classification/src/generate_dataset/dataset/79.4HA/79_4HA_k_test_new'
def text_save(filename, data):  #filename为写入CSV文件的路径，data为要写入数据列表.
    file = open(filename, 'a')
    for i in range(len(data)):
        s = str(int(data[i])).replace('[','').replace(']','')#去除[],这两行按数据不同，可以选择
        s = s+' '   #去除单引号，逗号，每行末尾追加换行符
        file.write(s)
    file.write('\n')
    file.close()
def takktile_zero():
    # helper python interface to be used elsewhere
    rospy.wait_for_service('/takktile/zero')
    try:
         zero = rospy.ServiceProxy('/takktile/zero', Empty)
         zero()
         return
    except rospy.ServiceException as e:
         print("Service call failed: %s"%e)
def printStatus(status):
    global pos
    pos = str(status.gPO)
    """Print the status string generated by the statusInterpreter function."""


def buffer_callback(data):
    global takktile_buffer_data
    takktile_buffer_data = data.data





def stability_test_delta(initial_data):
    global pos
    global a
    global touch_max
    global touch_max_position
    a.data = str(int(pos) + 1)
    pub.publish(a)
    while pos!=a.data:
        cur_pos = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)
    Touch_data = list(rospy.wait_for_message('takktile/calibrated', Touch, timeout=None).pressure)
    initial_data_mean = np.mean(initial_data)
    Touch_data = [abs(Touch_data[i] - initial_data[i]) for i in range(12)]
    #机械臂上提1mm
    raw_input('上移')
    Touch_data_up = list(rospy.wait_for_message('takktile/calibrated', Touch, timeout=None).pressure)

    Touch_data_up = [abs(Touch_data_up[i] - initial_data[i]) for i in range(12)]
    Touch_data_up_abs = list(map(lambda x:abs(x),Touch_data_up))
    print(Touch_data_up_abs)
    mean = [(Touch_data[i]-Touch_data_up[i]) for i in range(12)]
    mean_delta = [(Touch_data[i]-Touch_data_up[i])/(Touch_data[i]+0.000001) for i in range(12)]
    print('average contact data ',np.mean(Touch_data_up))
    if (touch_max-np.mean(Touch_data_up)) >5:
        print('pre_max_force:',touch_max)
        print('cur_force:',np.mean(Touch_data_up))
        return 2

    if np.mean(Touch_data_up)- initial_data_mean <1 and touch_max>6:
        return 2

    if np.mean(Touch_data_up)>touch_max:
        touch_max=np.mean(Touch_data_up)
        touch_max_position = a.data
    # the min mean force
    if np.mean(Touch_data_up_abs)>5:
        print(Touch_data_up,Touch_data)
        print('delta precent ',mean_delta)

        for i in range(12):
            if abs(mean_delta[i]) >0.5 and abs(mean[i])>5:
                print('drop>0.5')
                # ttset_pub.publish(0)
                return 0
        # ttset_pub.publish(1)
        return 1
    else:
        # ttset_pub.publish(0)
        print(Touch_data_up, Touch_data)
        print('np.mean(Touch_data_up_abs)<4.5')
        return 0

def stability_test_ttset(takktile_buffer_data_ini):
    global takktile_buffer_data
    takktile_buffer_data_pre = rospy.wait_for_message('takktile/data_buffer5',std_msgs.msg.Float64MultiArray).data
    rospy.sleep(0.065)
    takktile_buffer_data_final = rospy.wait_for_message('takktile/data_buffer5',std_msgs.msg.Float64MultiArray).data
    takktile_buffer_data_ini = list(takktile_buffer_data_ini[:-1])
    takktile_buffer_data_final = list(takktile_buffer_data_final[:-1])
    takktile_buffer_data_pre = list(takktile_buffer_data_pre[:-1])
    pmin_value = 1
    # print(takktile_buffer_data_ini,'1111111',len(takktile_buffer_data_ini),"22222",takktile_buffer_data_final,len(takktile_buffer_data_final))
    for i in range(12):
        ini_data = []
        final_data = []
        for j in range(5):
            ini_data.append(takktile_buffer_data_ini[12*j+i])
            final_data.append(takktile_buffer_data_final[12*j+i])
            j+=1
        # print(ini_data,final_data)
        ttest,pval=ttest_rel(ini_data,final_data)

        # print(pval)
        delta_percent = (sum(final_data) - sum(ini_data))/(sum(ini_data)+0.001)
        delta = sum(final_data) - sum(ini_data)
        if abs(delta_percent)>0.3 and abs(delta)>30 and pval < pmin_value:
            pmin_value = pval
            print(ini_data, final_data, sum(final_data), sum(ini_data), pval)

        i+=1
    print(pmin_value)
    if pmin_value<0.001:
        stability_test_ttset_result = 1
    else:
        stability_test_ttset_result = 0
    ttset_pub.publish(stability_test_ttset_result)
    return stability_test_ttset_result

def stability_test_ttset_neighbor():
    global takktile_buffer_data
    takktile_buffer_data_ini = rospy.wait_for_message('takktile/data_buffer5',std_msgs.msg.Float64MultiArray).data
    rospy.sleep(0.065)
    takktile_buffer_data_final = rospy.wait_for_message('takktile/data_buffer5',std_msgs.msg.Float64MultiArray).data
    takktile_buffer_data_ini = list(takktile_buffer_data_ini[:-1])
    takktile_buffer_data_final = list(takktile_buffer_data_final[:-1])
    pmin_value = 1
    # print(takktile_buffer_data_ini,'1111111',len(takktile_buffer_data_ini),"22222",takktile_buffer_data_final,len(takktile_buffer_data_final))
    for i in range(12):
        ini_data = []
        final_data = []
        for j in range(5):
            ini_data.append(takktile_buffer_data_ini[12*j+i])
            final_data.append(takktile_buffer_data_final[12*j+i])
            j+=1
        # print(ini_data,final_data)
        ttest,pval=ttest_rel(ini_data,final_data)
        print(ini_data,final_data,sum(final_data),pval)
        # print(pval)
        delta_percent = (sum(final_data) - sum(ini_data))/(sum(ini_data)+0.001)
        delta = sum(final_data) - sum(ini_data)
        if abs(delta_percent)>0.3 and abs(delta)>40 and pval < pmin_value:
            pmin_value = pval


        i+=1
    print(pmin_value)
    if pmin_value<0.001:
        stability_test_ttset_result = 1
    else:
        stability_test_ttset_result = 0
    ttset_pub.publish(stability_test_ttset_result)
    return stability_test_ttset_result

def contact_judge(initial_data,final_data):
    global touch_max
    delta_list = [abs(final_data[i] - initial_data[i]) for i in range(12)]
    a = np.mean(np.array(delta_list))
    print('force: ',a)
    if a > 6:
        touch_max = a
        return True
    return False

def listener(first_pos=None,initial_pos=None):
    global a
    global counter
    global initial_pos1
    global contact_or_not
    global pos
    global touch_max_position
    global touch_max

    raw_input('按ENTER开始.')

    contact_or_not = False

    # cur_pos = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)


    if initial_pos ==None:
        a.data = '20'
        pub.publish(a)

        while pos != a.data:
            cur_pos = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)
        rospy.sleep(0.2)
    else:
        a.data = initial_pos
        pub.publish(a)
        while pos != a.data:
            cur_pos = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)
    takktile_zero()

    # cur_pos = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)


    Touch_init = list(rospy.wait_for_message('takktile/calibrated', Touch, timeout=None).pressure)
    if first_pos ==None:
        a.data = str(input())
        initial_pos1 = str(int(a.data)-5)
        pub.publish(a)
        while pos != a.data:
            cur_pos = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)
    else:
        a.data = first_pos
        pub.publish(a)
        while pos != a.data:
            cur_pos = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)

    while contact_or_not is not True:
        # print(a.data)

        a.data = str(int(a.data)+1)
        pub.publish(a)
        # cur_pos = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)
        while pos != a.data:
            cur_pos = rospy.wait_for_message("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input)
        contact_touch = list(rospy.wait_for_message('takktile/calibrated', Touch, timeout=None).pressure)
        contact_or_not = contact_judge(Touch_init,contact_touch)
    print('Initial_Force')
    touch_max_position = a.data
    stability = 0


    #抓取稳定性分析
    while stability==0:
        stability = stability_test_delta(Touch_init)
        if stability ==2:
            print('drop')
            break
    if stability!=2:
        raw_input('first_stable.')
    else:
        raw_input('try again')
        stability = 0
        touch_max = 0
        listener(touch_max_position,initial_pos1)

    stability=0
    # rospy.sleep(0.5)
    raw_input('begin ttset')
    takktile_buffer_data_ini = rospy.wait_for_message('takktile/data_buffer5', std_msgs.msg.Float64MultiArray).data
    for i in range(20):
        # stability_cur = stability_test_ttset(takktile_buffer_data_ini)
        stability_cur = stability_test_ttset_neighbor()

        if stability_cur == 1:
            stability = stability_cur

    if stability == 1:
        print('unstable')
        raw_input('try again')
        stability = 0
        touch_max = 0
        listener(touch_max_position,initial_pos1)
    else:

        print('stable')







# END CALLBACK
if __name__ == '__main__':
    # plt.show()
    # BEGIN SUBSCRIBER
    rospy.init_node('topic_subscriber')
    pub = rospy.Publisher('robotiq/out/command', std_msgs.msg.String,queue_size=1)
    rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, printStatus)
    rospy.Subscriber('takktile/data_buffer5', std_msgs.msg.Float64MultiArray, buffer_callback)
    # ttset_pub = rospy.Publisher('ttest_result', std_msgs.msg.Float64, queue_size=1000)
    # ttset_pub.publish(0)
    ttset_pub = rospy.Publisher('ttest_result', std_msgs.msg.Float64, queue_size=1000)

    rospy.sleep(0.2)
    a = std_msgs.msg.String()
    touch_max=0
    touch_max_position=0
    initial_pos1 = None
    a.data='a'
    counter = 1
    pub.publish(a)
    while 1:
        listener()


