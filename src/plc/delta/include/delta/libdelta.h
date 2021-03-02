#pragma once 
#include <ros/ros.h>
#include <iostream>
#include "define.h"
#include "mbtcp/modbus.h"

DELTA_NAMESPACE_BEGIN
class libdelta: public modbus
{
private:
    /* data */
public:
    libdelta(ros::NodeHandlePtr nodeHandle);
    ~libdelta();
};
DELTA_NAMESPACE_END


