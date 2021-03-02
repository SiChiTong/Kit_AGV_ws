
#include "ros/ros.h"
#include "agv_define/agvlib.h"

using std::cout;
using namespace std;

void func(){
  ROS_INFO("main.cpp-func()");
}
namespace my_namespace {
    void test_namespace()
    {
        ROS_INFO("test_namespace");
        cout << "std::endl duoc su dung voi std!" << std::endl;
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testLib");
  ROS_INFO("main.cpp");

  func();

  uint8_t state_ = State(1);
  switch(state_){
    case STATE_EXECUTE:
      ROS_INFO("state: STATE_EXECUTE");
    break;
    case STATE_CANCEL:
      ROS_INFO("state: STATE_CANCEL");
    break;
    case STATE_DONE:
      ROS_INFO("state: STATE_DONE");
    break;
  }
  
  ros::spin();
  return 0;
}