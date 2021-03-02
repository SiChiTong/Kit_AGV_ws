#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <boost/thread/thread.hpp>
#include "ggm_motor/ggm_motor_md200.h"
#include "nav_converter/speed_wheel.h"

/* define struct */
typedef struct device{
   char _port[30];
   int _baud;
   int _ID;
}device_t;

/* Global varlue*/
uint8_t rate = 20; // Hz
device_t dv;
int16_t speed[] = {0,0};
ros::NodeHandlePtr nh;

/**********************************************************************
* Define Function 
***********************************************************************/
bool loadParam(ros::NodeHandlePtr nh, std::string node_name,device_t *dv);
void doStuff(uint8_t *publish_rate);
//Process ROS receive from navigation message, send to uController
void controlWheelCallback(const nav_converter::speed_wheel& robot)
{
	// ROS_INFO("blvd20km_controller.cpp-30-controlWheelCallback()");
	speed[0] = robot.wheel_letf;
  	speed[1] = robot.wheel_right;
} //navigationCallback
/**********************************************************************
* MAIN 
***********************************************************************/
int main(int argc,char **argv)
{
   /* create ros ndoe */
   ros::init(argc, argv, "GGM");
   nh = boost::make_shared<ros::NodeHandle>();
   std::string node_name = ros::this_node::getName();
   ROS_INFO("%s.cpp-node_name: %s", node_name.c_str(), node_name.c_str());
	if(loadParam(nh, node_name, &dv)){
		ROS_INFO("blvd20km_controller.cpp-Load parameter successfull");
	}else{
		ROS_ERROR("blvd20km_controller.cpp-Error when load parameter");
	}
   boost::thread Thread(doStuff,&rate);
   Thread.join();
   return 0;
}

/**********************************************************************
* Function detail 
***********************************************************************/
void doStuff(uint8_t *publish_rate)
{
   ros::Rate loop_rate(*publish_rate);
   /* Subscriber */
	ros::Subscriber control_wheel = nh->subscribe("control_wheel", 20, controlWheelCallback); 

   GGM::Driver_md200 *md200 = new GGM::Driver_md200(dv._port,dv._baud, PARITY_NONE, DATABIT_8, STOPBIT_1);
   uint8_t id[] = {1,2};
   md200->set_slave_id(id,sizeof(id)/sizeof(uint8_t));
   //md200->main_BC_ON();
   while(ros::ok())
   {
      md200->publish_speed(speed);
      loop_rate.sleep();
      ros::spinOnce();
   }
   ros::spin();
   //md200->main_BC_OFF();
   md200->close_port();
   delete(md200);
   ros::spin();  
}

/***************************************************************************/
bool loadParam(ros::NodeHandlePtr nh, std::string node_name, device_t *dv)
{
	ROS_INFO("loadParam() - node_name: %s",  node_name.c_str());

	if(!nh->param(node_name + "/baudrate", dv->_baud,dv->_baud)){
		return false;
	}
	ROS_INFO("baudrate: %d",dv->_baud);
	std::string port_str;    //port name
	if(!nh->param(node_name + "/port", port_str,port_str)){
		return false;
	}
   strcpy(dv->_port, port_str.c_str());
	ROS_INFO("port: %s",dv->_port);
	if(!nh->param(node_name + "/id", dv->_ID,dv->_ID)){
		return false;
	}
	ROS_INFO("ID: %d", dv->_ID);
	return true;
}