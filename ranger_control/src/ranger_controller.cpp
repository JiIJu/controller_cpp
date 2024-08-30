#include "ranger_control/ranger_controller.h"

int ranger_controller::cnt_sum = 0;
int ranger_controller::cnt_sum2 = 0;
double ranger_controller::theta_old = 0.0;
double ranger_controller::cnt_old = 0.0;

double ranger_controller::previous_u = 0.0;
double ranger_controller::previous_yaw_rate = 0.0;
double ranger_controller::d_hat = 1.0;
double ranger_controller::eyb_hat = 1.0;
double ranger_controller::w = 1.0;



// 이산 전달 함수와 미분의 계수 설정
std::vector<double> lpfNumerator = {0.0653};  // 전달 함수 분자 계수
std::vector<double> lpfDenominator = {1.0, -0.9347};  // 전달 함수 분모 계수

// Integrator 초기값 설정
double integratorGain = 1.0;  // 이득 값
double integratorInitialCondition = 0.0;  // 초기 조건
double integratorSampleTime = -1;  // 샘플링 시간 (-1은 불규칙 샘플링을 의미)

ranger_controller::ranger_controller(ros::NodeHandle &nh) 
    : nh_(nh), 
      derivativeBlock(dt.toSec()),  // 이산 미분 샘플링 시간 설정
      transferFunctionBlock(lpfNumerator, lpfDenominator),// 전달 함수 계수 초기화 
      integratorBlock(integratorGain, integratorInitialCondition, integratorSampleTime)
      {  

    L1 = 1.0;
    L2 = 1.0;
    
    get_topic();

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/four_wheel_steering_controller/cmd_vel", 10);

}

ranger_controller::~ranger_controller() {
}

void ranger_controller::controller(){
    double L = fabs(exb);
    if(L>20.0){
        L = 20.0;
    }
    else if(L<0.5){
        L = 0.5;
    }

    temp1 = L * w + d_hat;

    eyb_hat = integratorBlock.calculate(temp1);
    temp1 = (1-exp(-Kp * fabs(eyb))) * K;
    if(eyb>0){
        u2 = temp1 * 1;
    }
    else if(eyb<0) {
        u2 = temp1 * -1;
    }
    else{
        u2 = temp1 * 0;
    }
    ROS_INFO("u2 : %f" , u2);
    w = u2;

        geometry_msgs::Twist cmd_vel_msg;
    if(fabs(u2) < fabs(0.0001)){
        cmd_vel_msg.linear.x = 0.0; 
        cmd_vel_msg.linear.y = 0.0; 
        cmd_vel_msg.linear.z = 0.0; 

        cmd_vel_msg.angular.x = 0.0;
        cmd_vel_msg.angular.y = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        }
    else{
        cmd_vel_msg.linear.x = 0.5; 
        cmd_vel_msg.linear.y = 0.0; 
        cmd_vel_msg.linear.z = 0.0; 

        cmd_vel_msg.angular.x = 0.0;
        cmd_vel_msg.angular.y = 0.0;
        cmd_vel_msg.angular.z = w/L;
    }     
    ROS_INFO("w/L : %f" , w/L);
    cmd_vel_pub.publish(cmd_vel_msg);

}

void ranger_controller::get_frame(){
    pcheck(check , xr , yr , rrt_xr , rrt_yr);
    ex = x - referen_x;
    ey = y - referen_y;

    exb = cos(yaw) * ex + ey * sin(yaw);
    eyb = -sin(yaw) * ex + ey * cos(yaw); 

    e_yaw = atan2(-eyb,-exb);
}

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
    double u = yaw; 

    // 이산 미분 계산
    double derivative = derivativeBlock.calculate(u);  
    // 이산 전달 함수 계산
    yaw_rate = transferFunctionBlock.calculate(derivative);
    // ROS_INFO("previous_yaw_rate : %f" , previous_yaw_rate);
    // ROS_INFO("yaw_rate : %f" , yaw_rate);
    previous_u = u; 
    previous_yaw_rate = yaw_rate; 

    get_frame();

    yawd_cal(ex,ey);

    vehicle_yaw_func(yaw_d_ref, sum_theta2, cnt_sum2);

    controller();


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
        cnt_sum = cnt_old+ 1;
    } else if (dire == -1) {
        cnt_sum = cnt_old - 1;
    }
    else {
        cnt_sum = cnt_old;
    }

    if (cnt_sum == 0) {
        sum_theta = theta;
    } else {
        sum_theta = theta + cnt_sum * 2 * M_PI;
    }
    yaw = sum_theta;

    theta_old = theta;
    cnt_old = cnt_sum;
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
    Kp = g1;
    K = g2;
    Vr = vr;
}

void ranger_controller::yawd_cal(double ex, double& ey) {
    if(ex==0 && ey>=0){
        yaw_d_ref = M_PI / 2;
    }
    else if(ex == 0 && ey <0){
        yaw_d_ref = -M_PI / 2;
    }
    else{
        yaw_d_ref = atan2(-ey , -ex);
    }
}

void ranger_controller::vehicle_yaw_func(double theta, double& sum_theta2, int& cnt_sum2) {
    int dire=0;

    if (theta - theta_old < -6.1) {
        dire = 1;
    } else if (theta - theta_old > 6.1) {
        dire = -1;
    }

    if (dire == 1) {
        cnt_sum2 = cnt_old+ 1;
    } else if (dire == -1) {
        cnt_sum2 = cnt_old - 1;
    }
    else {
        cnt_sum2 = cnt_old;
    }

    if (cnt_sum2 == 0) {
        sum_theta2 = theta;
    } else {
        sum_theta2 = theta + cnt_sum2 * 2 * M_PI;
    }

    theta_old = theta;
    cnt_old = cnt_sum2;
  
    // 이산 전달 함수 계산
    yaw_d = transferFunctionBlock.calculate(sum_theta2);

    // 이산 미분 계산
    double derivative = derivativeBlock.calculate(sum_theta2);  
    // 이산 전달 함수 계산
    yaw_rate_d = transferFunctionBlock.calculate(derivative);
    // ROS_INFO("yaw_rate_d : %f" , yaw_rate_d);
}

void ranger_controller::updateDeltaTime(const ros::Duration& dt_) {
    dt = dt_;  
}
