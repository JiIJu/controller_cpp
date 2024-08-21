#include <ros/ros.h>
#include "ranger_control/ranger_controller.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "calculate_dt_node");
    ros::NodeHandle nh;

    
    ranger_controller ranger_controller(nh);

    ros::Rate rate(10);
    // while(ros::ok()){
    //     ros::spinOnce();
    //     rate.sleep();
    //    }
    ros::spin();
    
    return 0;
}