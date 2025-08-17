#include <ros/ros.h>
#include <Eigen/Eigen>
#include "quadrotor_dynamic.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qua_dynamic_node");
    ros::NodeHandle nh("~");


    AGEL::QuadrotorDynamic qua(0.710);


    Eigen::VectorXd x0(10), x1(10), x2(10), u(4);
std::cout << "sdf" << std::endl;

    x0 << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    u << 0, 0, 0, 0.720;
std::cout << "sdf" << std::endl;

    Eigen::Vector3d disturbance_acc(0, 0, 0);

    double dt = 0.01;

std::cout << "sdf" << std::endl;

    Eigen::Matrix<double, 10, 10> gx0, gx1;
    Eigen::Matrix<double, 10, 4> gu0, gu;

    Eigen::Matrix<double, 3, 1> acc;
    Eigen::Matrix<double, 3, 10> accdotx0;
    Eigen::Matrix<double, 3, 4> accdotu;



    qua.rk4_func(x0, u, disturbance_acc, dt, x1, gx0, gu0, acc, accdotx0, accdotu);
    qua.rk4_func(x1, u, disturbance_acc, dt, x2, gx1, gu, acc, accdotx0, accdotu);
    
    std::cout << x0 << "\n \n \n"<< std::endl;

    std::cout << x1 << "\n \n \n"<< std::endl;

    std::cout << gu0 << "\n \n \n"<< std::endl;

    std::cout << gu << "\n \n \n"<< std::endl;

    std::cout << gx0 << "\n \n \n"<< std::endl;
    std::cout << gx1 * gu << "\n \n \n"<< std::endl;






    ros::spin();

    ros::shutdown();


    return 0;
}