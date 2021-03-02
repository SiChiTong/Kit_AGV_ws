#ifndef NAVI_H
#define NAVI_H
#pragma once

#include <ros/ros.h>
#include <unistd.h>
#include <stdint.h>
#include <libraryparam/agvparam.h>

class navi
{
private:
	void agvGetparam(ros::NodeHandlePtr nodeHandle, agvParam &param);

public:
	agvParam param;
	speedWheel navigationConverter(cmd_vel msg);
	navi(ros::NodeHandlePtr nodeHandle);
	~navi();
};

navi::navi(ros::NodeHandlePtr nodeHandle)
{
	this->agvGetparam(nodeHandle, this->param);
}

navi::~navi() {}

speedWheel navi::navigationConverter(cmd_vel msg)
{
	speedWheel V;
	V.letf = (2 * msg.linear.x - msg.angular.z * this->param.L) / (2 * this->param.R) * this->param.K * RAD_PER_RPM;
	V.right = -(2 * msg.linear.x + msg.angular.z * this->param.L) / (2 * this->param.R) * this->param.K * RAD_PER_RPM;

	if (abs(V.letf) > this->param.speedMotor_max)
	{
		if (V.letf > 0)
			V.letf = this->param.speedMotor_max;
		else if (V.letf < 0)
			V.letf = -this->param.speedMotor_max;
	}

	if (abs(V.right) > this->param.speedMotor_max)
	{
		if (V.right > 0)
			V.right = this->param.speedMotor_max;
		else if (V.right < 0)
			V.right = -this->param.speedMotor_max;
	}
	if (abs(V.letf) < this->param.speedMotor_min)
		V.letf = 0;
	if (abs(V.right) < this->param.speedMotor_min)
		V.right = 0;

	return V;
}

void navi::agvGetparam(ros::NodeHandlePtr nodeHandle, agvParam &param)
{
	char paramName[70];
	sprintf(paramName, "/L");
	nodeHandle->getParam(paramName, param.L);
	ROS_INFO("%s = %f", paramName, param.L);
	sprintf(paramName, "/R");
	nodeHandle->getParam(paramName, param.R);
	ROS_INFO("%s = %f", paramName, param.R);
	sprintf(paramName, "/K");
	nodeHandle->getParam(paramName, param.K);
	ROS_INFO("%s = %f", paramName, param.K);
	sprintf(paramName, "/SpeedMotorMax");
	nodeHandle->getParam(paramName, param.speedMotor_max);
	ROS_INFO("%s = %d", paramName, param.speedMotor_max);
	sprintf(paramName, "/SpeedMotorMin");
	nodeHandle->getParam(paramName, param.speedMotor_min);
	ROS_INFO("%s = %d", paramName, param.speedMotor_min);
}

#endif