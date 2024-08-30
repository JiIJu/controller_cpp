#ifndef RANGER_CONTROLLER_H
#define RANGER_CONTROLLER_H

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>
#include <stdio.h>
#include <vector>  // 벡터 사용을 위한 헤더 추가
#include <cmath>


class DiscreteTimeIntegrator {
private:
    double Ts;              // 샘플링 시간
    double previousOutput;  // 이전 출력 값 (적분 값)
    double gain;            // 이득

public:
    // 새로운 생성자: 이득, 초기 조건, 샘플링 시간 초기화
    DiscreteTimeIntegrator(double gainValue, double initialCondition, double samplingTime) 
        : gain(gainValue), previousOutput(initialCondition), Ts(samplingTime) {}

    // 적분 계산 메서드
    double calculate(double currentInput) {
        // 이산 적분 계산: y[k] = y[k-1] + Ts * gain * u[k]
        double output = previousOutput + Ts * gain * currentInput;

        // 출력 값을 저장하여 다음 스텝에서 사용
        previousOutput = output;

        return output;
    }

    // 출력 값 초기화
    void reset() {
        previousOutput = 0.0;
    }
};

// 이산 미분 및 전달 함수 클래스 정의
class DiscreteDerivative {
private:
    double previousInput = 0.0;
    double Ts;

public:
    DiscreteDerivative(double samplingTime) : Ts(samplingTime) {}

    double calculate(double currentInput) {
        double derivative = (currentInput - previousInput) / Ts;
        previousInput = currentInput;
        return derivative;
    }
};

class DiscreteTransferFunction {
private:
    std::vector<double> numerator;
    std::vector<double> denominator;
    std::vector<double> inputHistory;
    std::vector<double> outputHistory;

public:
    DiscreteTransferFunction(const std::vector<double>& num, const std::vector<double>& den)
        : numerator(num), denominator(den) {
        inputHistory.resize(num.size(), 0.0);
        outputHistory.resize(den.size(), 0.0);
    }

    double calculate(double currentInput) {
        inputHistory.insert(inputHistory.begin(), currentInput);
        inputHistory.pop_back();

        double output = 0.0;

        for (size_t i = 0; i < numerator.size(); ++i) {
            output += numerator[i] * inputHistory[i];
        }

        for (size_t i = 1; i < denominator.size(); ++i) {
            output -= denominator[i] * outputHistory[i];
        }

        output /= denominator[0];
        outputHistory.insert(outputHistory.begin(), output);
        outputHistory.pop_back();

        return output;
    }
};

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

    ros::Publisher cmd_vel_pub;

    double vr;
    double g1;
    double g2;
    double L;
    double L1;
    double L2;

    double x;
    double y;
    double xr;
    double yr;
    double rrt_xr;
    double rrt_yr;
    int check;

    double ex;
    double ey;
    double exb;
    double eyb;
    double yaw;
    double yaw_rate;
    double yaw_raw;
    double quaternion[4];  
    double referen_x;
    double referen_y;
    double yaw_d_ref;
    double yaw_d;
    double yaw_rate_d;
    // double d_hat;
    double temp1;
    double temp2;
    double Kp;
    double K;
    double Vr;
    double u2;
    double e_yaw;

    static int cnt_sum;
    static double theta_old;
    static double cnt_old;
    static double d_hat;
    static double eyb_hat;
    static double w;
    double sum_theta;

    static int cnt_sum2;
    double sum_theta2;

    static double previous_u;
    static double previous_yaw_rate;

    ros::Duration dt;

    DiscreteDerivative derivativeBlock;  // 이산 미분 블록
    DiscreteTransferFunction transferFunctionBlock;  // 이산 전달 함수 블록
    DiscreteTimeIntegrator integratorBlock;

    void get_topic();
    void get_frame();

    void amcl_pose_stamp_callback(const geometry_msgs::PoseStamped::ConstPtr& amcl_pose_stamp_msg);
    void waypathR_callback(const geometry_msgs::PoseStamped::ConstPtr& waypathR_msg);
    void rrtR_callback(const geometry_msgs::PoseStamped::ConstPtr& rrtR_msg);
    void final_path_callback(const std_msgs::Int32::ConstPtr& final_path_msg);
    void odometry_callback(const nav_msgs::Odometry::ConstPtr &odometry_msg);
    void quaternionToEuler(const tf::Quaternion& quat, double& roll, double& pitch, double& yaw);

    void processAngle(double theta, double& sum_theta, int& cnt_sum);
    void pcheck(int u, double& way_x , double& way_y, double& rrt_x, double& rrt_y);
    void yawd_cal(double ex, double& ey);
    void vehicle_yaw_func(double theta, double& sum_theta, int& cnt_sum);\

    void controller();
};

#endif