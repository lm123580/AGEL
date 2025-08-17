#include <ros/ros.h>

#include "map_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh("~");

    AGEL::MapManager mg;

    mg.init(nh);

    // ros::spin();

    // ros::shutdown();
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
