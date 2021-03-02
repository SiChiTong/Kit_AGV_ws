#include "delta/libdelta.h"


DELTA_NAMESPACE::libdelta::~libdelta(){}
/**
 * Buid project
 * @param nodeHandle Node handle to get param for PLC
 */
DELTA_NAMESPACE::libdelta::libdelta(ros::NodeHandlePtr nodeHandle)
 : modbus(nodeHandle)
{
    ROS_INFO("PLC Delta is connecting.......");
    if(!this->modbus_connect()) 
        ROS_ERROR("PLC Delta conncetion error");
    else    
        ROS_INFO("PLC Delta connected");
}

