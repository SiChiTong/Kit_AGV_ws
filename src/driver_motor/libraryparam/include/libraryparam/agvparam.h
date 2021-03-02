
#ifndef LIBRARYPRAM_H
#define LIBRARYPRAM_H
#pragma once 

#include <ros/ros.h>
#include <unistd.h>
#include <stdint.h>

/*Value constant*/
#define PI 3.1415926535
#define RAD_PER_RPM 9.5492965964254

/* Parameter agv concept */
struct agvParam
{
    double L; 						/* Distance between two wheels */
    double R; 						/* wheel radius (in meters per radian)*/
    double K;  						/* Gear transmission*/
    int speedMotor_max;             /* Speed maximum of moter before gear */
    int speedMotor_min;             /* Speed maximum of moter before gear */
};

/* Parameter motor */
struct blvd20km
{
    int speed_max;
    int speed_min;
};

/* Parameter agv concept */
struct speedWheel{
    int letf;
    int right;
};

/* Geometry_msgs/Twist Message/ Linear*/
struct Linear
{
	double x;
	double y;
	double z;
};
/* Geometry_msgs/Twist Message/ Angular*/
struct Angular
{
	double x;
	double y;
	double z;
};

/* Geometry_msgs/Twist Message */
struct cmd_vel
{
	Linear linear;
	Angular angular;
};


#endif