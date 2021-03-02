#!/usr/bin/env python

from __future__ import print_function

from rospy_message_converter import json_message_converter
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import rospy

from agv_define.srv import convertJsonToRos_srv,convertJsonToRos_srvResponse

def convertJsonToRosCallback(req):
    print("convertJsonToRosServer.py - convertJsonToRosCallback() -> type: " + str(req.type))
    print("convertJsonToRosServer.py - convertJsonToRosCallback() -> dataJson: " + str(req.dataJson))
    message = json_message_converter.convert_json_to_ros_message(req.type, req.dataJson)
    print("convertJsonToRosServer.py - convertJsonToRosCallback() -> message: " + str(message))
    resp = convertJsonToRos_srvResponse() 
    if(req.type == "geometry_msgs/Twist"):
        resp.cmd_vel = message
    elif(req.type == "geometry_msgs/PoseStamped"):
        resp.pose_stamped = message
    return resp


if __name__ == "__main__":
    rospy.init_node('convertJsonToRosServer')
    s = rospy.Service('convertJsonToRosSrv', convertJsonToRos_srv, convertJsonToRosCallback)
    print("convertJsonToRosServer.py - Ready to convertJsonToRosSrv")
    rospy.spin()