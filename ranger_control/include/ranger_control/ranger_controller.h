#ifndef RANGER_CONTROLLER_H
#define RANGER_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

class ranger_controller {
public:
    
    ranger_controller(ros::NodeHandle &nh);
    virtual ~ranger_controller();

private:
    ros::NodeHandle nh_; 
    ros::Subscriber amcl_pose_stamp_sub;
    ros::Subscriber waypathR_sub;
    ros::Subscriber rrtR_sub;
    ros::Subscriber final_path_sub;
    ros::Subscriber odometry_sub;

    float x;
    float y;
    float xr;
    float yr;
    float rrt_xr;
    float rrt_yr;
    int check;

    // 멤버 함수 선언
    void get_topic();
    void amcl_pose_stamp_callback(const geometry_msgs::PoseStamped::ConstPtr& amcl_pose_stamp_msg);
    void waypathR_callback(const geometry_msgs::PoseStamped::ConstPtr& waypathR_msg);
    void rrtR_callback(const geometry_msgs::PoseStamped::ConstPtr& rrtR_msg);
    void final_path_callback(const std_msgs::Int32::ConstPtr& final_path_msg);

};


#endif