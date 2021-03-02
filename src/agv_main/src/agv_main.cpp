#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
// #include "geometry_msgs/Twist.h"
// #include "geometry_msgs/PoseStamped.h"
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
// #include "geometry_msgs/Quaternion.h"
// #include "move_base_msgs/MoveBaseActionResult.h"
// #include "sick_lidar_localization/SickLocResultPortTelegramMsg.h"

#include "agv_define/agvlib.h"
#include "agv_define/lift.h"
#include "agv_define/agv_action.h"
#include "agv_define/agv_flexisoft.h"
#include "agv_define/convertJsonToRos_srv.h"
#include <agv_define/lineAction.h> 
#include <actionlib/client/simple_action_client.h>

#define THRESHOLD_QUALITY_POSE 80

ros::Publisher cmd_vel_pub, pub_init_pose, pub_current_pose, pub_move_base_goal, pub_action_status, pub_ipc_to_fx, pub_ipc_fx_action_status;
ros::ServiceClient client_json_ros_srv, client_lift_srv;
agv_define::lift liftSrv;
agv_define::convertJsonToRos_srv convertJsonToRosSrv;
typedef actionlib::SimpleActionClient<agv_define::lineAction> Client_line_action;

bool isSystemGood   = false;   //Kiem tra trang thai an toan cua robot
bool isEnableAction = true;  
uint8_t quality = 0;
uint8_t action = 0;
std::string action_id = "";
// geometry_msgs::Twist cmd_vel;
// geometry_msgs::PoseStamped pose_stamped;
// geometry_msgs::PoseStamped agv_current_pose;

// void pubCmdvel(geometry_msgs::Twist cmd_vel_);
// void pubInitPose(geometry_msgs::PoseStamped pose_stamped_);
void pubIpcFxActionStatus(agv_define::agv_action action_ipc_fx_);
// void pubMoveBaseGoal(geometry_msgs::PoseStamped msg);
void callLineAction(agv_define::agv_action msg);
void callLiftService(agv_define::agv_action msg, std::string action_str);
void pubActionStatus(uint8_t status_);
void pubCurrentPose();
void pubIpcSystemState(bool state_);
// geometry_msgs::PoseStamped callConvertJsonToRosSrv(agv_define::agv_action msg);
// geometry_msgs::Quaternion getQuaternion(double yaw, double pitch, double roll);
void checkDevice();
// void stopRobot();
// void cancelAllAction();


void serverAgvActionCallback(const agv_define::agv_action& msg){
    ROS_INFO("agv_main.cpp- serverAgvActionCallback()");
    action = msg.action;
    action_id = msg.action_id;
    agv_define::agv_action action_ipc_fx_ = msg;
    action_ipc_fx_.status = 0;
    
    uint8_t action_state_ = State(msg.state);
    switch(action_state_){
        case STATE_CANCEL:
            isEnableAction = true;
            // cancelAllAction();
        break;
        case STATE_EXECUTE:
            uint8_t action_mode_ = ActionState(msg.action);
            switch(action_mode_){
                case ACTION_MANUAL:
                    ROS_INFO("agv_main.cpp- ACTION_MANUAL");
                    // callConvertJsonToRosSrv(msg);
                break;
                case ACTION_INITIAL_POSE:
                    ROS_INFO("agv_main.cpp- ACTION_INITIAL_POSE");
                    // pubActionStatus(1);
                    // pose_stamped = callConvertJsonToRosSrv(msg);
                    // pose_stamped.header = msg.header;
                    // pubInitPose(pose_stamped);
                break;
                case ACTION_CHARGING_IN:
                    ROS_INFO("agv_main.cpp- ACTION_CHARGING_IN");
                    action_ipc_fx_.max_vel_trans = -0.09;
                    callLineAction(msg);
                break;
                case ACTION_LIFT_IN:
                    ROS_INFO("agv_main.cpp- ACTION_LIFT_IN");
                    action_ipc_fx_.max_vel_trans = -0.09;
                    callLineAction(msg);
                break;
            }
        break;
    }
    pubIpcFxActionStatus(action_ipc_fx_);
}
void fxIpcActionStatusCallback(const agv_define::agv_action& msg){
    ROS_INFO("agv_main.cpp- fxIpcActionStatusCallback()");
    if((msg.action == action)&&(msg.action_id == action_id)){
        uint8_t action_status_ = Status(msg.status);
        pubActionStatus(action_status_);
        switch(action_status_){
            case STATUS_REJECTED:
                ROS_ERROR("agv_main.cpp- Flexisoft REJECTED the action");
            break;
            case STATUS_ACTIVE:
                uint8_t action_mode_ = ActionState(msg.action);
                switch(action_mode_){
                    case ACTION_ROTATE_GOAL:
                        ROS_INFO("agv_main.cpp- ACTION_ROTATE_GOAL");
                        // pose_stamped = callConvertJsonToRosSrv(msg);
                        // pubMoveBaseGoal(pose_stamped);
                    break;
                    case ACTION_MOVE_GOAL:
                        ROS_INFO("agv_main.cpp- ACTION_MOVE_GOAL");
                        // pose_stamped = callConvertJsonToRosSrv(msg);
                        // pubMoveBaseGoal(pose_stamped);
                    break;
                    case ACTION_CHARGING_IN:
                        ROS_INFO("agv_main.cpp- ACTION_CHARGING_IN");
                        callLineAction(msg);
                    break;
                    case ACTION_CHARGING_OUT:
                        ROS_INFO("agv_main.cpp- ACTION_CHARGING_OUT");
                        // pose_stamped = callConvertJsonToRosSrv(msg);
                        // pubMoveBaseGoal(pose_stamped);
                    break;
                    case ACTION_LIFT_IN:
                        ROS_INFO("agv_main.cpp- ACTION_LIFT_IN");
                        callLineAction(msg);
                    break;
                    case ACTION_LIFT_UP:
                        ROS_INFO("agv_main.cpp- ACTION_LIFT_UP");
                        callLiftService(msg, "LIFT_UP");
                    break;
                    case ACTION_LIFT_DOWN:
                        ROS_INFO("agv_main.cpp- ACTION_LIFT_DOWN");
                        callLiftService(msg, "LIFT_DOWN");
                    break;
                    case ACTION_LIFT_OUT:
                        ROS_INFO("agv_main.cpp- ACTION_LIFT_OUT");
                        // pose_stamped = callConvertJsonToRosSrv(msg);
                        // pubMoveBaseGoal(pose_stamped);
                    break;
                }
            break;
        }
    }
}
// void moveActionResultCallback(const move_base_msgs::MoveBaseActionResult& msg){
//     ROS_INFO("agv_main.cpp- moveResultCallback()");
//     uint8_t status_ = msg.status.status;
//     pubActionStatus(status_);
//     isEnableAction = true;
// }
// void currentPoseCallback(const sick_lidar_localization::SickLocResultPortTelegramMsg& msg)
// {
//     // ROS_INFO("agv_main.cpp- currentPoseCallback()");
//     quality = msg.telegram_payload.quality;
//     if(quality < THRESHOLD_QUALITY_POSE){
//         ROS_ERROR("agv_main.cpp->Quality is LOW: %d", quality);
//         isSystemGood = false;
//     }else{
//         ROS_INFO_ONCE("agv_main.cpp- Quality is GOOD!");
//     }

//     agv_current_pose.header = msg.header;
//     float x = float(msg.telegram_payload.posex)/1000;
//     float y = float(msg.telegram_payload.posey)/1000;
//     float th_degree = (msg.telegram_payload.poseyaw)/1000;
//     float th_rad = (th_degree*M_PI)/180;

//     agv_current_pose.pose.position.x = x;
//     agv_current_pose.pose.position.y = y;
//     agv_current_pose.pose.orientation = getQuaternion(th_rad, 0, 0);
// }
// void flexisoftCallback(const agv_define::agv_flexisoft& msg){
//     // ROS_INFO("agv_main.cpp- flexisoftCallback()");
//     if(msg.ST_SAFE_FC_ST_G == true){
//         isSystemGood = true;
//     }else{
//         isSystemGood = false;
//         stopRobot();
//         ROS_ERROR("agv_main.cpp- flexisoftCallback() -> System Flexisoft is FAILED");
//     }
// }


// void pubCmdvel(geometry_msgs::Twist cmd_vel_){
//     if(isEnableAction == true){
//         cmd_vel_pub.publish(cmd_vel_);
//         ROS_INFO("agv_main.cpp- Publish cmd_vel");
//     }else{
//         uint8_t status_ = 5;
//         pubActionStatus(status_);
//     }
// }
// void pubInitPose(geometry_msgs::PoseStamped pose_stamped_){
//     geometry_msgs::PoseWithCovarianceStamped init_pose_;
//     init_pose_.pose.pose = pose_stamped_.pose;
//     pub_init_pose.publish(init_pose_);
//     ROS_INFO("agv_main.cpp- Publish INIT_POSE");
// }
void pubIpcFxActionStatus(agv_define::agv_action action_ipc_fx_){
    pub_ipc_fx_action_status.publish(action_ipc_fx_);
    ROS_INFO("agv_main.cpp- IPC publish action status to Flexisoft");
}
// void pubMoveBaseGoal(geometry_msgs::PoseStamped msg){
//     if(isSystemGood && isEnableAction && (quality > THRESHOLD_QUALITY_POSE )){
//         isEnableAction = false;
//         pubCurrentPose();
//         pub_move_base_goal.publish(msg);
//         ROS_INFO("agv_main.cpp-pubMoveBaseGoal() -> Publish move_base goal");
//     }else{
//         ROS_ERROR("agv_main.cpp-quality: %d", quality);
//         uint8_t status_ = 5;
//         pubActionStatus(status_);
//     }
// }
void callLineAction(agv_define::agv_action msg){
    ROS_INFO("agv_main.cpp-218- callLineAction");
    Client_line_action client_line_action("line_action", true);
    client_line_action.waitForServer();
    agv_define::lineGoal lineGoal_;

    uint8_t state_ = State(msg.state);
    if(state_ == STATE_EXECUTE){
        if((isSystemGood == true)&&(isEnableAction == true)){
            lineGoal_.action = msg.action;
            client_line_action.sendGoal(lineGoal_);
            client_line_action.waitForResult();

            actionlib::SimpleClientGoalState status_ = client_line_action.getState();
            if(status_ == actionlib::SimpleClientGoalState::PENDING){
                ROS_INFO("agv_main.cpp- ACTION is %s", status_.toString().c_str());
                pubActionStatus(0);
            }else if(status_ == actionlib::SimpleClientGoalState::ACTIVE){
                ROS_INFO("agv_main.cpp- ACTION is %s", status_.toString().c_str());
                pubActionStatus(1);
            }else if(status_ == actionlib::SimpleClientGoalState::PREEMPTED){
                ROS_INFO("agv_main.cpp- ACTION is %s", status_.toString().c_str());
                pubActionStatus(2);
            }else if(status_ == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("agv_main.cpp- ACTION is %s", status_.toString().c_str());
                pubActionStatus(3);
            }else if(status_ == actionlib::SimpleClientGoalState::ABORTED){
                ROS_INFO("agv_main.cpp- ACTION is %s", status_.toString().c_str());
                pubActionStatus(4);
            }else if(status_ == actionlib::SimpleClientGoalState::REJECTED){
                ROS_INFO("agv_main.cpp- ACTION is %s", status_.toString().c_str());
                pubActionStatus(5   );
            }else if(status_ == actionlib::SimpleClientGoalState::RECALLED){
                ROS_INFO("agv_main.cpp- ACTION is %s", status_.toString().c_str());
                pubActionStatus(8);
            }else if(status_ == actionlib::SimpleClientGoalState::LOST){
                ROS_INFO("agv_main.cpp- ACTION is %s", status_.toString().c_str());
                pubActionStatus(9);
            }	
        }else{
            uint8_t status_ = 5;
            pubActionStatus(status_);
        }
    }else if(state_ == STATE_CANCEL){
        client_line_action.cancelAllGoals();
        ROS_INFO("agv_main.cpp- CANCEL Line action");
        // stopRobot();
    }
    isEnableAction = true; // Enable for next action
}
void callLiftService(agv_define::agv_action msg, std::string action_str){
    liftSrv.request.action = action_str;
    liftSrv.request.state = msg.state;
    geometry_msgs::PoseStamped pose_stamped_;
    if((isSystemGood == true)&&(isEnableAction == true)){
        if (client_lift_srv.call(liftSrv)){
            int8_t status_ = liftSrv.response.status;
            pubActionStatus(status_);
        }else{
            ROS_ERROR("Failed to call service convertJsonToRosSrv");
        }
        isEnableAction = true; // Enable for next action
    }else{
        uint8_t status_ = 5;
        pubActionStatus(status_);
    }
}
void pubActionStatus(uint8_t status_){
    // ROS_INFO("agv_main.cpp- pubActionStatus()");
    agv_define::agv_action action_status_;
    action_status_.action = action;
    action_status_.status = status_;
    action_status_.action_id = action_id;
    pub_action_status.publish(action_status_);
    ROS_INFO("agv_main.cpp-289 publish action status: %d", status_);
}
// void pubCurrentPose(){
//     pub_current_pose.publish(agv_current_pose);
//     ROS_INFO("agv_main.cpp- Publish agv_current_pose");
// }
// void pubIpcSystemState(bool state_){
//     agv_define::agv_flexisoft ipcState_msg;
//     ipcState_msg.IPC_SAFE_IO_IPC_Ok = state_;
//     pub_ipc_to_fx.publish(ipcState_msg);
// }

// geometry_msgs::PoseStamped callConvertJsonToRosSrv(agv_define::agv_action msg){
//     convertJsonToRosSrv.request.type = msg.type;
//     convertJsonToRosSrv.request.dataJson = msg.data;
//     geometry_msgs::PoseStamped pose_stamped_;
//     if (client_json_ros_srv.call(convertJsonToRosSrv)){
//         if(msg.type == "geometry_msgs/Twist"){
//             cmd_vel = convertJsonToRosSrv.response.cmd_vel;
//             pubCmdvel(cmd_vel);
//         }else if(msg.type == "geometry_msgs/PoseStamped"){
//             ROS_INFO("agv_main.cpp- ROS_TYPE: geometry_msgs/PoseStamped");
//             pose_stamped_ = convertJsonToRosSrv.response.pose_stamped;
//         }
//     }else{
//         ROS_ERROR("Failed to call service convertJsonToRosSrv");
//     }
//     return pose_stamped_;
// }
// geometry_msgs::Quaternion getQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
// {
//     // Abbreviations for the various angular functions
//     double cy = cos(yaw * 0.5);
//     double sy = sin(yaw * 0.5);
//     double cp = cos(pitch * 0.5);
//     double sp = sin(pitch * 0.5);
//     double cr = cos(roll * 0.5);
//     double sr = sin(roll * 0.5);
//     geometry_msgs::Quaternion q;
//     q.w = cr * cp * cy + sr * sp * sy;
//     q.x = sr * cp * cy - cr * sp * sy;
//     q.y = cr * sp * cy + sr * cp * sy;
//     q.z = cr * cp * sy - sr * sp * cy;
//     return q;
// }
// void stopRobot(){
//     cmd_vel.linear.x = 0;
//     cmd_vel.linear.y = 0;
//     cmd_vel.linear.z = 0;
//     cmd_vel.angular.x = 0;
//     cmd_vel.angular.y = 0;
//     cmd_vel.angular.z = 0;
//     cmd_vel_pub.publish(cmd_vel);
//     ROS_INFO("agv_main.cpp- STOP ROBOT");
// }
void checkDevice(){
    ROS_INFO("agv_main.cpp- checkDevice()");
    bool state_ = true;
    // pubIpcSystemState(state_);
}
// void cancelAllAction(){
//     ROS_INFO("agv_main.cpp- cancelAllAction()");
//     stopRobot();
//     agv_define::agv_action action_msg_;
//     action_msg_.action = action;
//     action_msg_.state = STATE_CANCEL;
//     switch(action){
//         case ACTION_CHARGING_IN:
//             callLineAction(action_msg_);
//         break;
//         case ACTION_LIFT_IN:
//             callLineAction(action_msg_);
//         break;
//     }
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agv_main");
    ros::NodeHandle n;
    ROS_INFO("agv_main.cpp");

    // system('sudo rm -rf /home/robotics/.ros/log');
    char *command_type = (char *)"sudo rm -rf /home/robotics/.ros/log";
    system(command_type);

    ros::Subscriber server_agv_action_sub = n.subscribe("/agv_action", 10, serverAgvActionCallback);
    ROS_INFO("agv_main.cpp- Subscriber topic /agv_action");
    // ros::Subscriber move_result_sub = n.subscribe("/move_base/result", 10, moveActionResultCallback);
    // ROS_INFO("agv_main.cpp- Subscriber topic /move_base/result");
    // ros::Subscriber current_pose_sub = n.subscribe("/sick_lidar_localization/driver/result_telegrams", 10, currentPoseCallback);
    // ROS_INFO("agv_main.cpp- Subscriber topic /sick_lidar_localization/driver/result_telegrams");
    // ros::Subscriber flexisoft_sub = n.subscribe("/fx_st_fc", 10, flexisoftCallback);
    // ROS_INFO("agv_main.cpp- Subscriber topic /fx_st_fc");
    // ros::Subscriber fx_ipc_action_sub = n.subscribe("/fx_ipc_action_status", 10, fxIpcActionStatusCallback);
    // ROS_INFO("agv_main.cpp- Subscriber topic /fx_ipc_action_status");

    client_json_ros_srv = n.serviceClient<agv_define::convertJsonToRos_srv>("convertJsonToRosSrv");
    client_lift_srv = n.serviceClient<agv_define::lift>("lift_srv");

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10); 
    ROS_INFO("agv_main.cpp- Publish topic /cmd_vel");
    // pub_init_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10); 
    // ROS_INFO("agv_main.cpp- Publish topic /initialpose");
    // pub_current_pose = n.advertise<geometry_msgs::PoseStamped>("/agv_current_pose", 10); 
    // ROS_INFO("agv_main.cpp- Publish topic /agv_current_pose");
    // pub_move_base_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10); 
    // ROS_INFO("agv_main.cpp- Publish topic /move_base_simple/goal");
    pub_action_status = n.advertise<agv_define::agv_action>("/action_status", 10); 
    ROS_INFO("agv_main.cpp- Publish topic /action_status");
    // pub_ipc_to_fx = n.advertise<agv_define::agv_flexisoft>("/ipc_st_io", 10); 
    // ROS_INFO("agv_main.cpp- Publish topic /ipc_st_io");
    pub_ipc_fx_action_status = n.advertise<agv_define::agv_action>("/ipc_fx_action_status", 10); 
    ROS_INFO("agv_main.cpp- Publish topic /ipc_fx_action_status");

    checkDevice();
  
    ros::spin();
    return 0;
}