#include <ros/ros.h>

#include "system_fsm.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "system_fsm");
    ros::NodeHandle nh;

    // 等待ros::Time初始化成功
    while (!ros::Time::isValid()) 
    {
        ROS_WARN("Waiting for valid time...");
        ros::Duration(0.05).sleep(); 
    }
    // cpu_set_t mask;
    // CPU_ZERO(&mask);
    // CPU_SET(8, &mask);
    // pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);
    
    xxx_xxx::SystemFSM system_fsm;
    system_fsm.init(nh);

    // ros::spin();
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}