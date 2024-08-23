#include "ranger_control/ranger_controller.h"

int ranger_controller::cnt_sum = 0;
double ranger_controller::theta_old = 0.0;

double ranger_controller::previous_u = 0.0;
double ranger_controller::previous_yaw_rate = 0.0;

ranger_controller::ranger_controller(ros::NodeHandle &nh) : nh_(nh) {

    L1 = 1.0;
    L2 = 1.0;

    get_topic();
}

ranger_controller::~ranger_controller() {

}

void ranger_controller::get_frame(){
    pcheck(check , xr , yr , rrt_xr , rrt_yr);
    ex = x - referen_x;
    ey = y - referen_y;

    exb = cos(yaw) * ex + ey * sin(yaw);
    eyb = -sin(yaw) * ex + ey * cos(yaw);

    ROS_INFO("exb : %f , eyb : %f" , exb,eyb);
}


// 토픽 subscribe
void ranger_controller::get_topic() {
    amcl_pose_stamp_sub = nh_.subscribe("amcl_pose_stamp", 10, &ranger_controller::amcl_pose_stamp_callback, this);
    waypathR_sub = nh_.subscribe("waypathR", 10, &ranger_controller::waypathR_callback, this);
    rrtR_sub = nh_.subscribe("rrtR", 10, &ranger_controller::rrtR_callback, this);
    final_path_sub = nh_.subscribe("final_path", 10, &ranger_controller::final_path_callback, this);
    odometry_sub = nh_.subscribe("odometry", 10, &ranger_controller::odometry_callback, this);
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

void ranger_controller::odometry_callback(const nav_msgs::Odometry::ConstPtr &odometry_msg){
    tf::Quaternion quat(
        odometry_msg->pose.pose.orientation.x,
        odometry_msg->pose.pose.orientation.y,
        odometry_msg->pose.pose.orientation.z,
        odometry_msg->pose.pose.orientation.w
    );

    double roll, pitch, yaw;
    quaternionToEuler(quat, roll, pitch, yaw);

    yaw_raw = yaw;

    processAngle(yaw_raw, sum_theta, cnt_sum);
  
    double dt_sec = dt.toSec();

    double K = 1.0;   
    // double Ts = 0.01;

    double u = yaw; 
    double derivative = K * (u - previous_u) / dt_sec;

    previous_u = u; 

    // 이 부분 확인 필요함
    
    // LPF3: c2d(7/(s+7), 0.01)
    double LPF3_numerator = 0.0653;
    double LPF3_denominator = -0.9347;

    yaw_rate = LPF3_numerator * derivative + LPF3_denominator * previous_yaw_rate;

    previous_yaw_rate = yaw_rate; 

    get_frame();



}

void ranger_controller::quaternionToEuler(const tf::Quaternion& quat, double& roll, double& pitch, double& yaw) {
    tf::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);
}

void ranger_controller::processAngle(double theta, double& sum_theta, int& cnt_sum) {
    int dire = 0;

    if (theta - theta_old < -4) {
        dire = 1;
    } else if (theta - theta_old > 4) {
        dire = -1;
    }

    if (dire == 1) {
        cnt_sum += 1;
    } else if (dire == -1) {
        cnt_sum -= 1;
    }

    if (cnt_sum == 0) {
        sum_theta = theta;
    } else {
        sum_theta = theta + cnt_sum * 2 * M_PI;
    }
    yaw = sum_theta;

    theta_old = theta;
    
}

void ranger_controller::pcheck(int u, double& way_x , double& way_y, double& rrt_x, double& rrt_y) {
    
    if(u==0){
        referen_x = rrt_x;
        referen_y = rrt_y;
        vr = 1;
        g1 = 3.5;
        g2 = 1;
    }
    else if(u==1){
        referen_x = way_x;
        referen_y = way_y;
        vr = 1;
        g1 = 3.5;
        g2 = 1;
    }
    else {
        referen_x = rrt_x;
        referen_y = rrt_y;
        vr = 0;
        g1 = 0;
        g2 = 0;
    }
    
}

void ranger_controller::updateDeltaTime(const ros::Duration& dt_) {
    dt = dt_;  
}