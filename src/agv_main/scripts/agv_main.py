#!/usr/bin/env python

import rospy
import os
import sys
import math
from rospy_message_converter import json_message_converter
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from sick_lidar_localization.msg import SickLocResultPortTelegramMsg

from agv_define.msg import agv_action
from agv_define.msg import agv_flexisoft
from agv_define.srv import lift

import roslib
roslib.load_manifest('agv_define')
import actionlib
from agv_define.msg import lineAction, lineGoal

pub_init_pose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
rospy.loginfo("agv_main.py-Publisher topic /initialpose")
pub_current_pose = rospy.Publisher('agv_current_pose', PoseStamped, queue_size=10)
rospy.loginfo("agv_main.py-Publisher topic /agv_current_pose")
pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.loginfo("agv_main.py-Publisher topic /cmd_vel")
pub_move_base_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
rospy.loginfo("agv_main.py-Publisher topic /move_base_simple/goal")
pub_action_status = rospy.Publisher('action_status', agv_action, queue_size=10)
rospy.loginfo("agv_main.py-Publisher topic /action_status")
pub_ipc_to_fx = rospy.Publisher('ipc_st_io', agv_flexisoft, queue_size=10)
rospy.loginfo("agv_main.py-Publisher topic /ipc_st_io")
pub_ipc_fx_action_status = rospy.Publisher('ipc_fx_action_status', agv_action, queue_size=10)
rospy.loginfo("agv_main.py-Publisher topic /ipc_fx_action_status")



action_ = 0
action_id_ = ""
agv_current_pose = PoseStamped()
isSystemGood = False

def actionMode(argument):
    switcher = {
        0:  "ACTION_FLOAT",
        1:  "ACTION_MANUAL",
        2:  "ACTION_INITIAL_POSE",
        3:  "ACTION_QUALITY_POSE",
        4:  "ACTION_ROTATE_GOAL",
        5:  "ACTION_MOVE_GOAL",
        6:  "ACTION_CHARGING_IN",
        7:  "ACTION_CHARGING_OUT",
        8:  "ACTION_LIFT_IN",
        9:  "ACTION_LIFT_UP",
        10: "ACTION_LIFT_DOWN",
        11: "ACTION_LIFT_OUT"
    }
    return switcher.get(argument, "Invalid action")

def currentPoseCallback(msg):
    # rospy.loginfo("agv_main.py-59-Subscriber topic /sick_lidar_localization/driver/result_telegrams: " + str(msg))
    agv_current_pose.header = msg.header
    x = float(msg.telegram_payload.posex)/1000
    y = float(msg.telegram_payload.posey)/1000
    th_degree = (msg.telegram_payload.poseyaw)/1000
    th_rad = (th_degree*math.pi)/180

    agv_current_pose.pose.position.x = x
    agv_current_pose.pose.position.y = y
    agv_current_pose.pose.orientation = getQuaternion(th_rad, 0, 0)
    # rospy.loginfo("agv_main.py-71- agv_current_pose: " + str(agv_current_pose))

# def serverAgvActionCallback(msg):
#     rospy.loginfo("agv_main.py-fxIpcActionStatusCallback() -> msg: " + str(msg)) 
#     global action_, action_id_, isSystemGood
#     if(isSystemGood == True):
#         action_ = msg.action
#         action_id_ = msg.action_id
#         action_mode = actionMode(msg.action)

#         rospy.loginfo("agv_main.py-action_mode: " + str(action_mode))   
#         if (action_mode == "ACTION_FLOAT"):
#             floatFunction()
#         elif (action_mode == "ACTION_MANUAL"):
#             message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
#             manualFunction(message)
#         elif (action_mode == "ACTION_INITIAL_POSE"):
#             message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
#             initPoseFunction(message)
#         elif (action_mode == "ACTION_QUALITY_POSE"):
#             message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
#             qualityPoseFunction(message)
#         elif (action_mode == "ACTION_ROTATE_GOAL"):
#             message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
#             moveGoalFunction(message)
#         elif (action_mode == "ACTION_MOVE_GOAL"):
#             message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
#             moveGoalFunction(message)
#         elif (action_mode == "ACTION_CHARGING_IN"):
#             lineFunction(msg, "LINE_IN")
#         elif (action_mode == "ACTION_CHARGING_OUT"):
#             # lineFunction(msg, "LINE_OUT")
#             message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
#             moveGoalFunction(message)
#         elif (action_mode == "ACTION_LIFT_IN"):
#             lineFunction(msg, "LINE_IN")
#         elif (action_mode == "ACTION_LIFT_UP"):
#             liftFunction(msg, "LIFT_UP")
#         elif (action_mode == "ACTION_LIFT_DOWN"):
#             liftFunction(msg, "LIFT_DOWN")
#         elif (action_mode == "ACTION_LIFT_OUT"):
#             # lineFunction(msg, "LINE_OUT")
#             message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
#             moveGoalFunction(message)
#     elif(isSystemGood == False):
#         rospy.loginfo("agv_main.py-127- System is NOT GOOD") 
def serverAgvActionCallback(msg):
    rospy.loginfo("agv_main.py-serverAgvActionCallback()") 
    global isSystemGood, pub_ipc_fx_action_status
    if(isSystemGood == True):
        action_msg = msg
        action_msg.status = 0 
         
        action_mode = actionMode(action_msg.action)
        # rospy.loginfo("agv_main.py-action_mode: " + str(action_mode)) 
        if (action_mode == "ACTION_FLOAT"):
            rospy.loginfo("agv_main.py-ACTION_FLOAT") 
        elif (action_mode == "ACTION_MANUAL"):
            message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
            manualFunction(message)
        elif (action_mode == "ACTION_INITIAL_POSE"):
            message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
            initPoseFunction(message)
        elif (action_mode == "ACTION_QUALITY_POSE"):
            rospy.loginfo("agv_main.py-ACTION_QUALITY_POSE") 
        elif (action_mode == "ACTION_ROTATE_GOAL"):
            rospy.loginfo("agv_main.py-ACTION_ROTATE_GOAL") 
        elif (action_mode == "ACTION_MOVE_GOAL"):
            rospy.loginfo("agv_main.py-ACTION_MOVE_GOAL") 
        elif (action_mode == "ACTION_CHARGING_IN"):
            action_msg.max_vel_trans = -0.09
        elif (action_mode == "ACTION_CHARGING_OUT"):
            rospy.loginfo("agv_main.py-ACTION_CHARGING_OUT") 
        elif (action_mode == "ACTION_LIFT_IN"):
            action_msg.max_vel_trans = -0.09
        elif (action_mode == "ACTION_LIFT_UP"):
            rospy.loginfo("agv_main.py-ACTION_LIFT_UP") 
        elif (action_mode == "ACTION_LIFT_DOWN"):
            rospy.loginfo("agv_main.py-ACTION_LIFT_DOWN") 
        elif (action_mode == "ACTION_LIFT_OUT"):
            rospy.loginfo("agv_main.py-ACTION_LIFT_OUT") 
        
        pub_ipc_fx_action_status.publish(action_msg)
        rospy.loginfo("agv_main.py-90- Publish to topic /ipc_fx_action_status -> action_msg: " + str(action_msg))
    elif(isSystemGood == False):
        rospy.loginfo("agv_main.py-96- System is NOT GOOD") 

def fxIpcActionStatusCallback(msg):
    rospy.loginfo("agv_main.py-fxIpcActionStatusCallback() -> msg: " + str(msg)) 
    global action_, action_id_, isSystemGood
    if(isSystemGood == True):
        action_ = msg.action
        action_id_ = msg.action_id
        action_mode = actionMode(msg.action)

        rospy.loginfo("agv_main.py-action_mode: " + str(action_mode))   
        if (action_mode == "ACTION_FLOAT"):
            floatFunction()
        elif (action_mode == "ACTION_MANUAL"):
            rospy.loginfo("agv_main.py- ACTION_MANUAL") 
        elif (action_mode == "ACTION_INITIAL_POSE"):
            rospy.loginfo("agv_main.py-ACTION_INITIAL_POSE") 
        elif (action_mode == "ACTION_QUALITY_POSE"):
            message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
            qualityPoseFunction(message)
        elif (action_mode == "ACTION_ROTATE_GOAL"):
            message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
            moveGoalFunction(message)
        elif (action_mode == "ACTION_MOVE_GOAL"):
            message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
            moveGoalFunction(message)
        elif (action_mode == "ACTION_CHARGING_IN"):
            lineFunction(msg)
        elif (action_mode == "ACTION_CHARGING_OUT"):
            # lineFunction(msg)
            message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
            moveGoalFunction(message)
        elif (action_mode == "ACTION_LIFT_IN"):
            lineFunction(msg)
        elif (action_mode == "ACTION_LIFT_UP"):
            liftFunction(msg, "LIFT_UP")
        elif (action_mode == "ACTION_LIFT_DOWN"):
            liftFunction(msg, "LIFT_DOWN")
        elif (action_mode == "ACTION_LIFT_OUT"):
            # lineFunction(msg)
            message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
            moveGoalFunction(message)
    elif(isSystemGood == False):
        rospy.loginfo("agv_main.py-127- System is NOT GOOD") 

def flexisoftCallback(msg):
    # rospy.loginfo("agv_main.py- flexisoftCallback -> msg: " + str(msg)) 
    global isSystemGood
    if(msg.ST_SAFE_FC_ST_G == True):
        isSystemGood = True
        # rospy.loginfo("agv_main.py- -> System Flexisoft is GOOD") 
    elif(msg.ST_SAFE_FC_ST_G == False):
        isSystemGood = False
        global pub_cmd_vel
        cmd_vel_ = Twist()
        pub_cmd_vel.publish(cmd_vel_)
        rospy.loginfo("agv_main.py-222 -> System Flexisoft is FAILED") 
        # rospy.loginfo("agv_main.py- STOP robot: " + str(cmd_vel_))

def floatFunction():
    print ("agv_main.py-Do Nothing")

def manualFunction(msg):
    print ("agv_main.py-manualFunction")
    global pub_cmd_vel
    pub_cmd_vel.publish(msg)
    rospy.loginfo("agv_main.py-pub_cmd_vel")

def qualityPoseFunction():
    print ("agv_main.py-qualityPoseFunction()")

def moveGoalFunction(msg):
    print ("agv_main.py-moveGoalFunction")
    pubCurrentPose()
    global pub_move_base_goal
    pubActionStatus(msg, 1)
    pub_move_base_goal.publish(msg)
    rospy.loginfo("agv_main.py-pub_move_base_goal")

def initPoseFunction(msg):
    print ("agv_main.py-initPoseFunction")
    global pub_init_pose
    pubActionStatus(msg, 1)
    initPose = PoseWithCovarianceStamped()
    initPose.header = msg.header
    initPose.pose.pose = msg.pose
    pub_init_pose.publish(initPose)

def lineFunction(msg):
    print ("agv_main.py-124-lineFunction")
    pubActionStatus(msg, 1)
    goal = lineGoal()
    goal.action = msg.action
    if(msg.state == 0):
        client.send_goal(goal)
        client.wait_for_result()
        result_ = client.get_state()
        rospy.loginfo("agv_main.py-133- line result: " + str(result_))
        pubActionStatus(msg, result_)
    elif(msg.state == 1):
        client.cancel_all_goals()
        rospy.loginfo("agv_main.py-137- cancel_all_goals")

def liftFunction(msg, action_str):
    print ("agv_main.py-liftFunction")
    pubActionStatus(msg, 1)
    rospy.wait_for_service('lift_srv')
    try:
        lift_srv = rospy.ServiceProxy('lift_srv', lift)
        res = lift_srv(action_str, msg.state)
        print ("agv_main.py-res.status: " + str(res.status))
        pubActionStatus(msg, res.status)
        return res.status
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def moveResult(msg):
    rospy.loginfo("agv_main.py-149-moveResult(): " + str(msg.status.status))
    status_ = msg.status.status
    pubActionStatus(msg, status_)

def pubActionStatus(msg, status):
    global pub_action_status, action_, action_id_
    action_status_ = agv_action()
    action_status_.action = action_
    action_status_.status = status
    action_status_.action_id = action_id_
    # rospy.loginfo("agv_main.py-action_status: " + str(action_status_))
    pub_action_status.publish(action_status_)
    rospy.loginfo("agv_main.py-161-publish action_status: " + str(action_status_))

def pubCurrentPose():
    pub_current_pose.publish(agv_current_pose)
    rospy.loginfo("agv_main.py-171-publish /agv_current_pose: " + str(agv_current_pose))

def getQuaternion(yaw, pitch, roll): # yaw (Z), pitch (Y), roll (X)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

def checkDevice():
    rospy.loginfo("agv_main.py-checkDevice()")
    state_ = True
    pubIpcSystemState(state_)

def pubIpcSystemState(state_):
    ipcState_msg = agv_flexisoft()
    ipcState_msg.IPC_SAFE_IO_IPC_Ok = state_
    pub_ipc_to_fx.publish(ipcState_msg)
    # rospy.loginfo("agv_main.py- Publish to topic /ipc_to_fx: " + str(ipcState_msg))

if __name__ == '__main__':
    rospy.init_node('agv_main', log_level=rospy.DEBUG)

    os.system('sudo rm -rf /home/robotics/.ros/log')
    rospy.loginfo("agv_main.py- $sudo rm -rf /home/robotics/.ros/log")

    # os.system('sudo ip link set can0 type can')
    # os.system('sudo ip link set can0 up type can bitrate 125000')
    # rospy.loginfo("agv_main.py-Config Can port")

    rospy.Subscriber("agv_action", agv_action, serverAgvActionCallback)
    rospy.loginfo("agv_main.py-Subscriber topic /agv_action")

    client = actionlib.SimpleActionClient('line_action', lineAction)
    client.wait_for_server()

    rospy.Subscriber("/move_base/result", MoveBaseActionResult, moveResult)
    rospy.loginfo("agv_main.py-Subscriber topic /move_base/result")

    rospy.Subscriber("sick_lidar_localization/driver/result_telegrams", SickLocResultPortTelegramMsg, currentPoseCallback)
    rospy.loginfo("agv_main.py-Subscriber topic /sick_lidar_localization/driver/result_telegrams")

    rospy.Subscriber("/fx_st_fc", agv_flexisoft, flexisoftCallback)
    rospy.loginfo("agv_main.py-Subscriber topic /fx_st_fc")

    rospy.Subscriber("/fx_ipc_action_status", agv_action, fxIpcActionStatusCallback)
    rospy.loginfo("agv_main.py-Subscriber topic /fx_ipc_action_status")
    
    checkDevice()

    rospy.spin()