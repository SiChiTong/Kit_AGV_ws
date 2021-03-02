#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sstream>
#include <iostream>  
#include <linelibrary/agvline.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>
#include <agvlinepkg/MLS_Measurement.h>
#include <agv_define/agv_action.h>
#include <actionlib/server/simple_action_server.h>
#include <agv_define/lineAction.h>  // Note: "Action" is appended
#include <agv_define/agvlib.h>

/* creat struct*/
Mlse_info Forward_Info;
Mlse_info Backward_Info;
cmd_vel Line_cmd_vel;

/*global variable */
uint8_t rate = 20; // Hz
agvline *agvLine;
uint8_t action_;
bool isEnable = NULL; 
bool succeed = NULL;
int direct = 0;
/***Create Node***/
ros::NodeHandlePtr nh;
ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;
/**********************************************************************
 *                   		Define Function 
***********************************************************************/
/* Create function Call back*/
void mlsForwardCallback(const agvlinepkg::MLS_Measurement &msg);
void mlsBackwardCallback(const agvlinepkg::MLS_Measurement &msg);
void std_msgsCallback(const std_msgs::String::ConstPtr& msg);

/* Event function */
void agvSucceed();
void areaOne();
void areaTwo();
void areaThree();
/* Create Thread */
boost::thread lineThread; 
void agvRunForward_doStuff(uint8_t *publish_rate);
void agvRunBackward_doStuff(uint8_t *publish_rate);
void agvRunOnDinhGocForward_doStuff(uint8_t *publish_rate);
void agvRunOnDinhGocBackward_doStuff(uint8_t *publish_rate);
/**********************************************************************
 *                   		MAIN 
***********************************************************************/
int main(int argc, char **argv)
{
	/*** ROS initialization ***/
	ros::init(argc, argv, "agvkitLine");
    //ros::Rate loop_rate(rate);
    // ROS_INFO("agvLineAction.cpp- main()");
    
    // char *command_type = (char *)"sudo ip link set can0 type can";
    // system(command_type);
    char *command_baud = (char *)"sudo ip link set can0 up type can bitrate 125000";
    system(command_baud);
    ROS_INFO("agvLineAction.cpp-336- System command config CAN");

	/***Create Node***/
    nh = boost::make_shared<ros::NodeHandle>();

    /* Subscriber position line */
	ros::Subscriber mlsForward = nh->subscribe("mls0", 20, mlsForwardCallback);
	ros::Subscriber mlsBackward = nh->subscribe("mls1", 20, mlsBackwardCallback);
    ros::Subscriber velCallback = nh->subscribe("chatter", 20, std_msgsCallback);
	/* Publisher */
	cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 20);

    /* Create Class agvline */
	agvLine = new agvline(nh, (const uint8_t)rate);
	agvLine->areaOne = &areaOne;
	agvLine->areaTwo = &areaTwo;
	agvLine->areaThree = &areaThree;
    ROS_INFO("agvlinectrl.cpp::42 ->AGV line is ready !!");
	ros::spin();
	delete(agvLine);
	return 0;
}

/**********************************************************************
 *                   		Function detail 
***********************************************************************/
/* fix code*/
void mlsForwardCallback(const agvlinepkg::MLS_Measurement &msg)
{
	Forward_Info.status = msg.status;
	Forward_Info.lcp = msg.lcp;
	for (int i = 0; i < 3; i++)
		Forward_Info.position[i] = msg.position[i];
} //mlsForwardCallback

void mlsBackwardCallback(const agvlinepkg::MLS_Measurement &msg)
{
	Backward_Info.status = msg.status;
	Backward_Info.lcp = msg.lcp;
	for (int i = 0; i < 3; i++)
		Backward_Info.position[i] = msg.position[i];
} //mlsForwardCallback

void std_msgsCallback(const std_msgs::String::ConstPtr& msg)
{
    std::stringstream geek(msg->data.c_str());
    geek >> direct;
    isEnable = false;
    usleep(10000);
    if(lineThread.joinable() == true) lineThread.join();
    usleep(10000);
    if(lineThread.joinable() == false) 
    {
        if(direct == 1)
        {
            lineThread = boost::thread(agvRunForward_doStuff,&rate);    
        }
        else if(direct == -1)
        {
            lineThread = boost::thread(agvRunBackward_doStuff,&rate);
        }
        else if(direct == 2)
        {
            lineThread = boost::thread(agvRunOnDinhGocForward_doStuff,&rate);
        }
        else if(direct == -2)
        {
            lineThread = boost::thread(agvRunOnDinhGocBackward_doStuff,&rate);
        }
        
    }
} //cmd_velCallback

void agvRunOnDinhGocForward_doStuff(uint8_t *publish_rate)
{
    agvLine->EnventEnable(FOR_WARD);
    ROS_INFO("agvLineAction.cpp :120 -> line_forward is ready!!");
    ros::Rate loop_rate(*publish_rate);
    isEnable = true;
    while (ros::ok() && isEnable==true)
    {        
        agvLine->getDataAgvforward(Forward_Info);
        agvLine->getDataAgvbackward(Backward_Info);
        //ROS_INFO("agvLineAction.cpp-125-Forward Line track level is %d", agvLine->agvForwardInfo.status.track_level);
        if(agvLine->agvForwardInfo.status.line_good == false)
        {
            //isEnable = false;
            //ROS_ERROR("agvLineAction.cpp-158 - Line is NOT good");
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
        } 
        else
        {
            Line_cmd_vel = agvLine->agvLineOnDinhGocForward();
            cmd_vel.linear.x = Line_cmd_vel.linear.x;
            cmd_vel.angular.z = Line_cmd_vel.angular.z;
        }
        
        cmd_vel_pub.publish(cmd_vel);	
        // if(succeed)
        // {
        //     isEnable = false;
        // }
        loop_rate.sleep();
		ros::spinOnce();
    }
    agvLine->bockAcceleration(agvLine->acceLineForward);
    agvLine->EnventDisEnable();
}

void agvRunOnDinhGocBackward_doStuff(uint8_t *publish_rate)
{
    agvLine->EnventEnable(BACK_WARD);
    ROS_INFO("agvLineAction.cpp :147 ->line_backward is ready!!");
    ros::Rate loop_rate(*publish_rate);
    isEnable = true;
    while (ros::ok() && isEnable==true)
    {
        agvLine->getDataAgvforward(Forward_Info);
        agvLine->getDataAgvbackward(Backward_Info);
        //ROS_INFO("agvLineAction.cpp-155-Backward Line track level is %d", agvLine->agvBackwardInfo.status.track_level);
        if(agvLine->agvBackwardInfo.status.line_good == false)
        {
            //isEnable = false;
            //ROS_ERROR("agvLineAction.cpp-158 - Line is NOT good");
            cmd_vel.linear.x =0;
            cmd_vel.angular.z =0;
        } 
        else
        {
            Line_cmd_vel = agvLine->agvLineOnDinhGocBackward();
            cmd_vel.linear.x = Line_cmd_vel.linear.x;
            cmd_vel.angular.z = Line_cmd_vel.angular.z;
        }
        cmd_vel_pub.publish(cmd_vel);	
        // if(succeed)
        // {
        //     isEnable = false;
        // }
        loop_rate.sleep();
        ros::spinOnce();
    } 
    agvLine->bockAcceleration(agvLine->acceLineBackward);
    agvLine->EnventDisEnable();       
}

void agvRunForward_doStuff(uint8_t *publish_rate)
{
    agvLine->EnventEnable(FOR_WARD);
    ROS_INFO("agvLineAction.cpp :120 -> line_forward is ready!!");
    ros::Rate loop_rate(*publish_rate);
    isEnable = true;
    while (ros::ok() && isEnable==true)
    {        
        agvLine->getDataAgvforward(Forward_Info);
        agvLine->getDataAgvforward(Forward_Info);
        //ROS_INFO("agvLineAction.cpp-125-Forward Line track level is %d", agvLine->agvForwardInfo.status.track_level);
        if(agvLine->agvForwardInfo.status.line_good == false)
        {
            //isEnable = false;
            //ROS_ERROR("agvLineAction.cpp-158 - Line is NOT good");
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
        } 	
        else
        {
            Line_cmd_vel = agvLine->agvLineforward();
            cmd_vel.linear.x = Line_cmd_vel.linear.x;
            cmd_vel.angular.z = Line_cmd_vel.angular.z;
        }
        
        cmd_vel_pub.publish(cmd_vel);
        // if(succeed)
        // {
        //     isEnable = false;
        // }
        loop_rate.sleep();
		ros::spinOnce();
    }
    agvLine->bockAcceleration(agvLine->acceLineForward);
    agvLine->EnventDisEnable();
}

void agvRunBackward_doStuff(uint8_t *publish_rate)
{
    agvLine->EnventEnable(BACK_WARD);
    ROS_INFO("agvLineAction.cpp :147 ->line_backward is ready!!");
    ros::Rate loop_rate(*publish_rate);
    isEnable = true;
    while (ros::ok() && isEnable==true)
    {
        agvLine->getDataAgvforward(Forward_Info);
        agvLine->getDataAgvbackward(Backward_Info);
        //ROS_INFO("agvLineAction.cpp-155-Backward Line track level is %d", agvLine->agvBackwardInfo.status.track_level);
        if(agvLine->agvBackwardInfo.status.line_good == false)
        {
            //isEnable = false;
            //ROS_ERROR("agvLineAction.cpp-158 - Line is NOT good");
            cmd_vel.linear.x =0;
            cmd_vel.angular.z =0;
        } 
        else
        {
            Line_cmd_vel = agvLine->agvLinebackward();
            cmd_vel.linear.x = Line_cmd_vel.linear.x;
            cmd_vel.angular.z = Line_cmd_vel.angular.z;
        }
        cmd_vel_pub.publish(cmd_vel);	
        // if(succeed)
        // {
        //     isEnable = false;
        // }
        loop_rate.sleep();
        ros::spinOnce();
    } 
    agvLine->bockAcceleration(agvLine->acceLineBackward);
    agvLine->EnventDisEnable();       
}


void agvSucceed()
{
    sleep(1);
    succeed = true;
}
/* USER CODE EVENT CALLBACK*/
void areaOne()
{	
  /* Do some thing*/
	//ROS_INFO("RUNNING!!");
}
void areaTwo()
{
	/* Do some thing*/
	//ROS_INFO("DETECTING!!");
}
void areaThree()
{   
	/* Do some thing*/
	//ROS_INFO("STOPPING!!");
    //agvLine -> agvStop();
    //agvSucceed();
}