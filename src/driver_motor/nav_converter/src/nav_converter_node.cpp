#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unistd.h>
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>
#include "nav_converter/speed_wheel.h"
#include "nav_converter/nav_converter.h"


typedef enum EXIT_CODES_ENUM
{
  EXIT_OK = 0,
  EXIT_ERROR = 1,
  EXIT_CONFIG_ERROR 
} EXIT_CODES;
template<class T> static T readMember(XmlRpc::XmlRpcValue & value, const std::string & member, const T & defaultvalue)
{
  if(value.hasMember(member))
    return value[member];
  return defaultvalue;
}
/*global variable */
clock_t start;
double timeoutMs = 1; //sec
static cmd_vel cmdVel;
static speedWheel W;
static navi *naviConvert;
static uint8_t rate = 20; // Hz

/* Create function Call back*/
void cmd_velCallback(const geometry_msgs::Twist &msg);
/* Create Function Thread */
void navgationDoStuff(ros::NodeHandlePtr nh, uint8_t *publish_rate);
void checkCallback();

int main(int argc, char **argv)
{
    /* Khoi tao Node */
    ros::init(argc, argv, "nav_converter");
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
    ROS_INFO("nav_converter.cpp::42 -> is ready !!");
    /*Create thread*/
	boost::thread nav_converterThread(navgationDoStuff,nh, &rate);
	nav_converterThread.join();
    ros::spin();
    delete(naviConvert);
    return EXIT_OK;
}
/**********************************************************
                    Function detail
**********************************************************/
void cmd_velCallback(const geometry_msgs::Twist &msg)
{
    start = clock();
    cmdVel.linear.x = msg.linear.x;
    cmdVel.angular.z = msg.angular.z;
    // ROS_INFO("%f %f",cmdVel.linear.x,cmdVel.angular.z);
} //cmd_velCallback

void navgationDoStuff(ros::NodeHandlePtr nh, uint8_t *publish_rate)
{
    naviConvert = new navi(nh);
    /* Publisher */
    ros::Publisher Navigation_control = nh->advertise<nav_converter::speed_wheel>("control_wheel", 20);
    nav_converter::speed_wheel motor;
    /* Subscriber */
    ros::Subscriber velCallback = nh->subscribe("cmd_vel", 20, cmd_velCallback);
    ROS_INFO("nav_converter.cpp:59 -> navgationDoStuff is ready!!");
    start = clock();
    ros::Rate loop_rate(*publish_rate);
    while (ros::ok())
    {
        W = naviConvert->navigationConverter(cmdVel);
        motor.wheel_letf = W.letf;
        motor.wheel_right = W.right;
        ROS_INFO_STREAM("V = " << cmdVel.linear.x << " W = " << cmdVel.angular.z <<" Banh trai = " << W.letf << " Banh phai = " << W.right);
        Navigation_control.publish(motor);
        loop_rate.sleep();
        ros::spinOnce();
    }
}

void checkCallback(){
    if((double)(clock() - start)/CLOCKS_PER_SEC >= timeoutMs) 
    {
        W.letf = W.right= 0;
        start = clock();
    }
}