#!/usr/bin/env python
import rospy
import time
import pandas as pd
from std_msgs.msg import String
from agvlinepkg.msg import MLS_Measurement
from geometry_msgs.msg import Twist

dataSave_mls0 = 0
dataSave_mls1 = 0
x = 0
z = 0

def callbackmls0(data):
    global dataSave_mls0
    header_ = False
    dataSave_mls0 = data.position[1]
    # rospy.loginfo(" -> error: %f(m)", dataSave_)
    # exportCsv(0, dataSave_, header_)

def callbackmls1(data):
    global dataSave_mls1
    header_ = False
    dataSave_mls1 = data.position[1]
    # rospy.loginfo(" -> error: %f(m)", dataSave_)
    # exportCsv(0, dataSave_, header_)

def cmdvelcallback(data):
    global x, z
    x = data.linear.x
    z = data.angular.z

def exportCsv(seconds_, mls0_, mls1_, x, z, header_):
    # rospy.loginfo("exportCsv.py-11 - mls0_: %lf", mls0_)
    # rospy.loginfo("exportCsv.py-11 - mls1_: %lf", mls1_)
    t = rospy.Time.from_sec(time.time())
    seconds_ = t.to_sec() #floating point
    content = {'time(s)': [seconds_], 'mls0_(m)': [mls0_], 'mls1_(m)': [mls1_], 'V(m/s)': [x], 'W(rad/s)': [z]}
    df = pd.DataFrame(content, columns= ['time(s)', 'mls0_(m)', 'mls1_(m)', 'V(m/s)', 'W(rad/s)'])
    df.to_csv (r'./export.csv',mode = 'a', index = False, header=header_)
    # print ("Saved file error_pos1_mls1")
    
def subsciberTopic(topicName_, type_):
    rospy.Subscriber(topicName_, type_, callback)
    rospy.loginfo("exportCsv.py-11 - Subscriber topic %s", topicName_)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('exportCsv', anonymous=True)
    rospy.loginfo("exportCsv.py")
    rate =rospy.Rate(20)

    global dataSave_mls0, dataSave_mls1, x, z
    
    exportCsv(0, 0,0,0,0, True)

    # topicName_0 = "/mls0"    
    # type_ = MLS_Measurement
    # subsciberTopic(topicName_0, type_)

    # topicName_1 = "/mls1"    
    # type_ = MLS_Measurement
    # subsciberTopic(topicName_1, type_)
    
    rospy.Subscriber("/mls0", MLS_Measurement, callbackmls0)
    rospy.Subscriber("/mls1", MLS_Measurement, callbackmls1)
    rospy.Subscriber("/cmd_vel", Twist, cmdvelcallback)
    

    while not rospy.is_shutdown():
      exportCsv(0, dataSave_mls0, dataSave_mls1, x, z, False)
      rate.sleep()
    