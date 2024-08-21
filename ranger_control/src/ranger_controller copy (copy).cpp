#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;




int main(int argc, char** argv) {
    ros::init(argc, argv, "calculate_dt_node");
    ros::NodeHandle nh;


    ros::Time prev_time = ros::Time::now();

    ros::Rate loop_rate(100); 

    while (ros::ok()) {

        ros::Time current_time = ros::Time::now();

  
        ros::Duration dt = current_time - prev_time;
        ROS_INFO("dt: %f seconds", dt.toSec());


        prev_time = current_time;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
