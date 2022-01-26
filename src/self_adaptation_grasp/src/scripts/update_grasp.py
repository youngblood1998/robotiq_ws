#!/usr/bin/env python
# -*- coding:utf-8 -*-

import roslib;roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep
import std_msgs
import rospy
import moveit_commander

global slip_arr
global contact_flag
global contact_print
global pos
global command
global nowPos
global arm

slip_arr = []
contact_flag = False
contact_print = True
command = outputMsg.Robotiq2FGripper_robot_output()
arm = moveit_commander.MoveGroupCommander("manipulator")

CONTACT_FORCE = 0.2
SLIP_MAX = 2
INIT_POS = 85
nowPos = INIT_POS

def genCommand(char, command):
    """Update the command according to the character entered by the user."""    
        
    if char == 'a':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 50
        command.rFR  = 150

    if char == 'r':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 0

    if char == 'c':
        command.rPR = 255

    if char == 'o':
        command.rPR = 0   

    #If the command entered is a int, assign this value to rPRA
    try: 
        command.rPR = int(char)
        if command.rPR > 255:
            command.rPR = 255
        if command.rPR < 0:
            command.rPR = 0
    except ValueError:
        pass                    
        
    if char == 'f':
        command.rSP += 25
        if command.rSP > 255:
            command.rSP = 255
            
    if char == 'l':
        command.rSP -= 25
        if command.rSP < 0:
            command.rSP = 0

            
    if char == 'i':
        command.rFR += 25
        if command.rFR > 255:
            command.rFR = 255
            
    if char == 'd':
        command.rFR -= 25
        if command.rFR < 0:
            command.rFR = 0

    return command


def setStatus(data):
    global pos
    pos = int(data.gPO)

def arm_move():
    global arm
    
    pose = arm.get_current_pose()
    pose.pose.position.z += 0.25
    arm.set_pose_target(pose)
    arm.go(wait=True)

    pose = arm.get_current_pose()
    pose.pose.position.z -= 0.2
    arm.set_pose_target(pose)
    arm.go(wait=True)

    pose = arm.get_current_pose()
    pose.pose.position.x += 0.14
    pose.pose.position.y += 0.14
    arm.set_pose_target(pose)
    arm.go(wait=True)

    pose = arm.get_current_pose()
    pose.pose.position.x -= 0.14
    pose.pose.position.y -= 0.14
    arm.set_pose_target(pose)
    arm.go(wait=True)

    pose = arm.get_current_pose()
    pose.pose.position.x -= 0.14
    pose.pose.position.y += 0.14
    arm.set_pose_target(pose)
    arm.go(wait=True)

    pose = arm.get_current_pose()
    pose.pose.position.x += 0.14
    pose.pose.position.y -= 0.14
    arm.set_pose_target(pose)
    arm.go(wait=True)

def pressure_callback(data):
    global nowPos
    global pos
    global command
    global contact_flag
    global contact_print

    if contact_flag:
        if contact_print:
            print "contact!!!"
            arm_move()
            contact_print = False
        return 0
    else:
        pressure_data = data.data
        left_f = 0
        right_f = 0
        for i in range(6):
            left_f += (pressure_data[i])**2
            right_f +=(pressure_data[i+6])**2
        if left_f**(0.5) < CONTACT_FORCE and right_f**(0.5) < CONTACT_FORCE:
            if nowPos <= pos:
                nowPos += 2
                command = genCommand(str(nowPos), command)
                pub.publish(command)
        else:
            contact_flag = True


def ttest_callback(data):
    global nowPos
    global pos
    global command
    global contact_flag
    global slip_arr

    if contact_flag:
        if len(slip_arr) < 5:
            slip_arr.append(data.data)
        else:
            del slip_arr[0]
            slip_arr.append(data.data)
        if sum(slip_arr) >= SLIP_MAX:
            nowPos += 1
            command = genCommand(str(nowPos), command)
            pub.publish(command)
            print "slip!!!"
            del slip_arr[:]
    else:
        return 0



if __name__ == '__main__':
    global arm
    rospy.init_node('robotiq_control')

    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    command = genCommand('r', command)
    pub.publish(command)
    sleep(0.5)
    command = genCommand('a', command)
    pub.publish(command)

    arm.set_pose_reference_frame("base_link")
    arm.set_goal_position_tolerance(0.005)
    arm.set_goal_orientation_tolerance(0.005)
    arm.set_max_velocity_scaling_factor(1)
    arm.set_max_acceleration_scaling_factor(1)
    arm.allow_replanning(True)
    arm.set_planning_time(50)

    # raw_input("press enter to start")

    command = genCommand(str(INIT_POS), command)
    pub.publish(command)

    rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input,setStatus)
    sleep(1)
    rospy.Subscriber('/pressure', std_msgs.msg.Float64MultiArray, pressure_callback)
    rospy.Subscriber('ttest_result', std_msgs.msg.Float64, ttest_callback)

    print "start!!!"
    while not rospy.is_shutdown():
        rospy.spin()