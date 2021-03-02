#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

# BURGER_MAX_LIN_VEL = 0.22
# BURGER_MAX_ANG_VEL = 2.84
MAX_LIN_VEL = 1.0
MAX_ANG_VEL = 3.5
MIN_LIN_VEL = 0.00
MIN_ANG_VEL = 0.0

# WAFFLE_MAX_LIN_VEL = 0.26
# WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your KIT_AGV!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input > 0:
        if input < low:
          input = low
        elif input > high:
          input = high
        else:
          input = input
    elif input < 0:
        if input > -low:
          input = -low
        elif input < -high:
          input = -high
        else:
          input = input

    return input

# def checkLinearLimitVelocity(vel):
#     if turtlebot3_model == "burger":
#       vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
#     elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
#       vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
#     else:
#       vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
#     return vel

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, MIN_LIN_VEL, MAX_LIN_VEL)
    return vel

# def checkAngularLimitVelocity(vel):
#     if turtlebot3_model == "burger":
#       vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
#     elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
#       vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
#     else:
#       vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
#     return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, MIN_ANG_VEL, MAX_ANG_VEL)
    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('kit_agv_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    pub_str = rospy.Publisher('chatter', String, queue_size=10)
    # turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    control_mode = 1
    direct = 0

    try:
        print(msg)
        while not rospy.is_shutdown():

            key = getKey()
            if key == '1': # che do bang tay
                control_mode = 1
                direct = "0"
                pub_str.publish(direct)
                print("che do kieu khien bang tay qua cac nut an w, x, a, d, s")
                #print("/sai so:%f(m)/sai so bam theo:%f/van toc goc:%f(rad/s)/ van toc dai:%f(m/s)/",e,e_follow,result.angular.z,result.linear.x)
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                twist = Twist()
                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0.0
                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                pub.publish(twist)
            elif key == '2': #che navigaton
                control_mode = 2
                direct = "1"
                pub_str.publish(direct)
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print("thuat toan so 1 - di chuyen tien")
                twist = Twist()
                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0.0
                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                pub.publish(twist)
            elif key == '3': # che do do line
                control_mode = 3
                direct = "-1"
                pub_str.publish(direct)
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print("thuat toan so 1 - di chuyen lui ")
                twist = Twist()
                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0.0
                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                pub.publish(twist)

            elif key == '4': # che do do line
                control_mode = 4
                direct = "2"
                pub_str.publish(direct)
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print("thuan toan so 2 - di chuyen tien")
                twist = Twist()
                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0.0
                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                pub.publish(twist)
            elif key == '5': # che do do line
                control_mode = 5
                direct = "-2"
                pub_str.publish(direct)
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print("thuat toan so 2 - di chuyen lui")
                twist = Twist()
                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0.0
                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                pub.publish(twist)

            if control_mode == 1:    
                if key == 'w' or key == 'W':
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                    twist = Twist()
                    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                    twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0.0
                    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                    pub.publish(twist)
                elif key == 'x' or key == 'X':
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                    twist = Twist()
                    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                    twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0.0
                    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                    pub.publish(twist) 
                elif key == 'a' or key == 'A':
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                    twist = Twist()
                    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                    twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0.0
                    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                    pub.publish(twist) 
                elif key == 'd' or key == 'D':
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(vels(target_linear_vel,target_angular_vel))
                    twist = Twist()
                    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                    twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0.0
                    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                    pub.publish(twist) 
                elif key == ' ' or key == 's' or key == 'S':
                    target_linear_vel   = 0.0
                    control_linear_vel  = 0.0
                    target_angular_vel  = 0.0
                    control_angular_vel = 0.0
                    print(vels(target_linear_vel, target_angular_vel))
                    twist = Twist()
                    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                    twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0.0
                    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                    pub.publish(twist)                    
                else:
                    if (key == '\x03'):
                        break

            if status == 20 :
                print(msg)
                status = 0



    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
