#include <ros/ros.h>
#include "delta/libdelta.h"

/* Global varlue*/
uint8_t rate = 20; // Hz
ros::NodeHandlePtr nh;

int main(int argc, char** argv)
{
    /* create ros ndoe */
    ros::init(argc, argv, "plc_control");
    nh = boost::make_shared<ros::NodeHandle>();
    DELTA_NAMESPACE::libdelta *delta_plc = new DELTA_NAMESPACE::libdelta(nh);

    ros::spin();
    delta_plc->modbus_close();
    delete(delta_plc);
    return 0;
}