#include "ranger_control/ranger_controller.h"

ranger_controller::ranger_controller(ros::NodeHandle &nh) : nh_(nh) {
    get_topic();  
}

ranger_controller::~ranger_controller() {
    
}


// 토픽 subscribe
void ranger_controller::get_topic() {
    amcl_pose_stamp_sub = nh_.subscribe("amcl_pose_stamp", 10, &ranger_controller::amcl_pose_stamp_callback, this);
    waypathR_sub = nh_.subscribe("waypathR", 10, &ranger_controller::waypathR_callback, this);
    rrtR_sub = nh_.subscribe("rrtR", 10, &ranger_controller::rrtR_callback, this);
    final_path_sub = nh_.subscribe("final_path", 10, &ranger_controller::final_path_callback, this);
}

void ranger_controller::amcl_pose_stamp_callback(const geometry_msgs::PoseStamped::ConstPtr& amcl_pose_stamp_msg) {
    x = amcl_pose_stamp_msg->pose.position.x;
    y = amcl_pose_stamp_msg->pose.position.y;
}

void ranger_controller::waypathR_callback(const geometry_msgs::PoseStamped::ConstPtr& waypathR_msg) {
    xr = waypathR_msg->pose.position.x;
    yr = waypathR_msg->pose.position.y;
}

void ranger_controller::rrtR_callback(const geometry_msgs::PoseStamped::ConstPtr& rrtR_msg) {
    rrt_xr = rrtR_msg->pose.position.x;
    rrt_yr = rrtR_msg->pose.position.y;
}

void ranger_controller::final_path_callback(const std_msgs::Int32::ConstPtr& final_path_msg){
    check = final_path_msg->data;
}