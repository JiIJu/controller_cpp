#ifndef RANGER_CONTROLLER_H
#define RANGER_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
class ranger_controller {
public:
    
    ranger_controller(ros::NodeHandle &nh);
    virtual ~ranger_controller();

    void updateDeltaTime(const ros::Duration& dt);

private:
    ros::NodeHandle nh_; 
    ros::Subscriber amcl_pose_stamp_sub;
    ros::Subscriber waypathR_sub;
    ros::Subscriber rrtR_sub;
    ros::Subscriber final_path_sub;
    ros::Subscriber odometry_sub;

    // 토픽 변수
    double x;
    double y;
    double xr;
    double yr;
    double rrt_xr;
    double rrt_yr;
    int check;

    // 프레임 변수
    double ex;
    double ey;
    double exb;
    double eyb;
    double yaw;
    double yaw_rate;
    double yaw_raw;
    double quaternion[4];  


    // matlab function 변수
    static int cnt_sum;
    static double theta_old;
    double sum_theta;

    static double previous_u;
    static double previous_yaw_rate;

    ros::Duration dt;

    void get_topic();
    void get_frame();


    void amcl_pose_stamp_callback(const geometry_msgs::PoseStamped::ConstPtr& amcl_pose_stamp_msg);
    void waypathR_callback(const geometry_msgs::PoseStamped::ConstPtr& waypathR_msg);
    void rrtR_callback(const geometry_msgs::PoseStamped::ConstPtr& rrtR_msg);
    void final_path_callback(const std_msgs::Int32::ConstPtr& final_path_msg);
    void odometry_callback(const nav_msgs::Odometry::ConstPtr &odometry_msg);
    void quaternionToEuler(const tf::Quaternion& quat, double& roll, double& pitch, double& yaw);

    void processAngle(double theta, double& sum_theta, int& cnt_sum);
};


#endif