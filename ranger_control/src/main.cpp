#include <ros/ros.h>
#include "ranger_control/ranger_controller.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "ranger_controller");
    ros::NodeHandle nh;

    
    ros::Time prev_time = ros::Time::now();
    
    ranger_controller ranger_controller(nh);
 
    ros::Rate rate(100);
    while(ros::ok()){
        
        ros::Time current_time = ros::Time::now();

        ros::Duration dt = current_time - prev_time;
        ranger_controller.updateDeltaTime(dt);
        prev_time = current_time;
        // ROS_INFO("current_time : %f", current_time.toSec());
        // ROS_INFO("dt : %f", dt.toSec());
        ros::spinOnce();

        rate.sleep();
       }
    

    ros::spin();
    
    return 0;
}