#ifndef _QUADROTOR_DYNAMIC_H_
#define _QUADROTOR_DYNAMIC_H_

#include <iostream>
#include <vector>
#include <Eigen/Eigen>

namespace AGEL
{
    #define PX 0
    #define PY 1
    #define PZ 2
    #define VX 3
    #define VY 4
    #define VZ 5
    #define QW 6
    #define QX 7
    #define QY 8
    #define QZ 9
    #define RX 0
    #define RY 1
    #define RZ 2
    #define TH 3

    class QuadrotorDynamic
    {
    public:
        static constexpr int x_dim_ = 10;
        static constexpr int u_dim_ = 4;
        double hover_ratio_;

    private:
        Eigen::VectorXd aux;
        Eigen::VectorXd xdot1;
        Eigen::VectorXd xdot2;
        Eigen::VectorXd xdot3;
        Eigen::VectorXd xdot4;
        Eigen::Matrix<double, x_dim_, x_dim_> xd1gx0;
        Eigen::Matrix<double, x_dim_, x_dim_> xd2gx1;
        Eigen::Matrix<double, x_dim_, x_dim_> xd3gx2;
        Eigen::Matrix<double, x_dim_, x_dim_> xd4gx3;
        Eigen::Matrix<double, x_dim_, u_dim_> xd1gu;
        Eigen::Matrix<double, x_dim_, u_dim_> xd2gu;
        Eigen::Matrix<double, x_dim_, u_dim_> xd3gu;
        Eigen::Matrix<double, x_dim_, u_dim_> xd4gu;
        Eigen::Matrix<double, x_dim_, x_dim_> x1gx0;
        Eigen::Matrix<double, x_dim_, x_dim_> x2gx0;
        Eigen::Matrix<double, x_dim_, x_dim_> x3gx0;
        Eigen::Matrix<double, x_dim_, u_dim_> x1gu;
        Eigen::Matrix<double, x_dim_, u_dim_> x2gu;
        Eigen::Matrix<double, x_dim_, u_dim_> x3gu;
        Eigen::Matrix<double, x_dim_, x_dim_> iden;


    public:
        QuadrotorDynamic();
        QuadrotorDynamic(double hover_ratio);
        ~QuadrotorDynamic();


        void xdot_func( const Eigen::VectorXd &x, const Eigen::VectorXd &u, Eigen::VectorXd &xdot, 
                        Eigen::Matrix<double, x_dim_, x_dim_> &gx, 
                        Eigen::Matrix<double, x_dim_, u_dim_> &gu);
        
        void rk4_func(  const Eigen::VectorXd &x0, const Eigen::VectorXd &u, const double &dt, Eigen::VectorXd &x1, 
                        Eigen::Matrix<double, x_dim_, x_dim_> &gx0, Eigen::Matrix<double, x_dim_, u_dim_> &gu);
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}















#endif