#include "px4_interface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "b_node");
    ros::NodeHandle nh("~");

    AGEL::Px4Interface px4(nh);

    ros::Rate rate(50);

    // px4初始化
    px4.set_px4_mode("AUTO.LOITER");
    sleep(1);
    px4.arm();
    sleep(1);
    px4.set_px4_mode("OFFBOARD");


    auto time_start = ros::Time::now();

    while ((ros::Time::now() - time_start).toSec() < 10.0)
    {
        double x = 0.0, y = 0.0, z = 2.0, yaw = 0.0;
        px4.set_pos(x, y, z, yaw);
        
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("OK!!!");
    while (ros::ok())
    {
        // auto start_time = ros::Time::now();
        
        std::cout << px4.get_vel()[2] << std::endl;

        double rx = 0.0, ry = 0.0, rz = 0.0, trust = 0.710;
        px4.set_rate_with_trust(rx, ry, rz, trust);

        ros::spinOnce();
        rate.sleep();
    }
    
    ros::shutdown();

    return 0;
}