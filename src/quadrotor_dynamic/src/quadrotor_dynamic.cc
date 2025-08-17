#include "quadrotor_dynamic.h"

namespace AGEL
{
    QuadrotorDynamic::QuadrotorDynamic() :
    hover_ratio_(0.710), aux(x_dim_), xdot1(x_dim_), xdot2(x_dim_), xdot3(x_dim_), xdot4(x_dim_)
    {
        iden = Eigen::MatrixXd::Identity(x_dim_, x_dim_);
        
        aux.setZero();
        xdot1.setZero();
        xdot2.setZero();
        xdot3.setZero();
        xdot4.setZero();
        
        xd1gx0.setZero();
        xd2gx1.setZero();
        xd3gx2.setZero();
        xd4gx3.setZero();
        
        xd1gu.setZero();
        xd2gu.setZero();
        xd3gu.setZero();
        xd4gu.setZero();
        
        x1gx0.setZero();
        x2gx0.setZero();
        x3gx0.setZero();

        x1gu.setZero();
        x2gu.setZero();
        x3gu.setZero();
    }


    QuadrotorDynamic::QuadrotorDynamic(double hover_ratio) :
    hover_ratio_(hover_ratio), aux(x_dim_), xdot1(x_dim_), xdot2(x_dim_), xdot3(x_dim_), xdot4(x_dim_)
    {
        iden = Eigen::MatrixXd::Identity(x_dim_, x_dim_);
        
        aux.setZero();
        xdot1.setZero();
        xdot2.setZero();
        xdot3.setZero();
        xdot4.setZero();
        
        xd1gx0.setZero();
        xd2gx1.setZero();
        xd3gx2.setZero();
        xd4gx3.setZero();
        
        xd1gu.setZero();
        xd2gu.setZero();
        xd3gu.setZero();
        xd4gu.setZero();
        
        x1gx0.setZero();
        x2gx0.setZero();
        x3gx0.setZero();

        x1gu.setZero();
        x2gu.setZero();
        x3gu.setZero();
    }

    QuadrotorDynamic::~QuadrotorDynamic() = default;


    void QuadrotorDynamic::xdot_func(   const Eigen::VectorXd &x, 
                                        const Eigen::VectorXd &u, 
                                        Eigen::VectorXd &xdot, 
                                        Eigen::Matrix<double, x_dim_, x_dim_> &gx, 
                                        Eigen::Matrix<double, x_dim_, u_dim_> &gu) 
    {
        // const double &px = x(0);
        // const double &py = x(1);
        // const double &pz = x(2);
        const double &vx = x(3);
        const double &vy = x(4);
        const double &vz = x(5);
        const double &qw = x(6);
        const double &qx = x(7);
        const double &qy = x(8);
        const double &qz = x(9);
        const double &rx = u[0];
        const double &ry = u[1];
        const double &rz = u[2];
        const double thrust = u[3] * 9.8066 / hover_ratio_;
        
        xdot << vx, vy, vz, 
                2 * (qx * qz + qw * qy) * thrust, 
                2 * (qy * qz - qw * qx) * thrust, 
                ((1 - 2 * (qx * qx + qy * qy)) * thrust - 9.8066),
                0.5 * (-rx * qx - ry * qy - rz * qz), 
                0.5 * (rx * qw + rz * qy - ry * qz), 
                0.5 * (ry * qw - rz * qx + rx * qz), 
                0.5 * (rz * qw + ry * qx - rx * qy);
        gx(PX, VX) = 1;
        gx(PY, VY) = 1;
        gx(PZ, VZ) = 1;
        gx(VX, QX) = 2 * qz * thrust, gx(VX, QY) = 2 * qw * thrust, gx(VX, QZ) = 2 * qx * thrust, gx(VX, QW) = 2 * qy * thrust;
        gx(VY, QX) = -2 * qw * thrust, gx(VY, QY) = 2 * qz * thrust, gx(VY, QZ) = 2 * qy * thrust, gx(VY, QW) = -2 * qx * thrust;
        gx(VZ, QX) = -4 * qx * thrust, gx(VZ, QY) = -4 * qy * thrust;
        gx(QW, QX) = 0.5 * -rx, gx(QW, QY) = 0.5 * -ry, gx(QW, QZ) = 0.5 * -rz;
        gx(QX, QW) = 0.5 * rx, gx(QX, QY) = 0.5 * rz, gx(QX, QZ) = 0.5 * -ry;
        gx(QY, QW) = 0.5 * ry, gx(QY, QX) = 0.5 * -rz, gx(QY, QZ) = 0.5 * rx;
        gx(QZ, QW) = 0.5 * rz, gx(QZ, QX) = 0.5 * ry, gx(QZ, QY) = 0.5 * -rx;
        
        gu(VX, TH) = 2 * (qx * qz + qw * qy) * 9.8066 / hover_ratio_;
        gu(VY, TH) = 2 * (qy * qz - qw * qx) * 9.8066 / hover_ratio_;
        gu(VZ, TH) = (1 - 2 * (qx * qx + qy * qy)) * 9.8066 / hover_ratio_;
        gu(QW, RX) = 0.5 * -qx, gu(QW, RY) = 0.5 * -qy, gu(QW, RZ) = 0.5 * -qz;
        gu(QX, RX) = 0.5 * qw, gu(QX, RY) = 0.5 * -qz, gu(QX, RZ) = 0.5 * qy;
        gu(QY, RX) = 0.5 * qz, gu(QY, RY) = 0.5 * qw, gu(QY, RZ) = 0.5 * -qx;
        gu(QZ, RX) = 0.5 * -qy, gu(QZ, RY) = 0.5 * qx, gu(QZ, RZ) = 0.5 * qw;
    }


    // x1 = f_rk4(x0, u, dt), gx0 = dx1 / dx0, gu = dx1 / du
    void QuadrotorDynamic::rk4_func(const Eigen::VectorXd &x0, const Eigen::VectorXd &u, const double &dt, Eigen::VectorXd &x1, 
                                    Eigen::Matrix<double, x_dim_, x_dim_> &gx0, Eigen::Matrix<double, x_dim_, u_dim_> &gu)
    {
        xdot_func(x0, u, xdot1, xd1gx0, xd1gu);
        aux = x0 + xdot1 * dt / 2.0;
        x1gx0 = iden + dt * 1 / 2.0 * xd1gx0;
        x1gu = dt / 2.0 * xd1gu;
        xdot_func(aux, u, xdot2, xd2gx1, xd2gu);
        aux = x0 + xdot2 * dt / 2.0;
        x2gx0 = iden + dt * 1 / 2.0 * xd2gx1 * x1gx0;
        x2gu = dt / 2.0 * (xd2gx1 * x1gu + xd2gu);
        xdot_func(aux, u, xdot3, xd3gx2, xd3gu);
        aux = x0 + xdot3 * dt;
        x3gx0 = iden + dt * xd3gx2 * x2gx0;
        x3gu = dt * (xd3gx2 * x2gu + xd3gu);
        xdot_func(aux, u, xdot4, xd4gx3, xd4gu);
        x1 = x0 + dt * (1 / 6. * xdot1 + 1 / 3. * xdot2 + 1 / 3. * xdot3 + 1 / 6. * xdot4);
        
        gx0 = iden + 
            dt * (1 / 6.0 * xd1gx0
            + 1 / 3.0 * xd2gx1 * x1gx0
            + 1 / 3.0 * xd3gx2 * x2gx0
            + 1 / 6.0 * xd4gx3 * x3gx0
            );
        gu = dt * (1 / 6.0 * xd1gu + 1 / 3.0 * (xd2gu + xd2gx1 * x1gu) + 1 / 3.0 * (xd3gu + xd3gx2 * x2gu) + 1 / 6.0 * (xd4gu + xd4gx3 * x3gu));
    }
}