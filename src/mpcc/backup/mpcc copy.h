#ifndef MPCC_H_
#define MPCC_H_

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <memory>
#include <nlopt.hpp>
#include <Eigen/Eigen>
#include <chrono>

namespace AGEL
{
    class QuadrotorDynamic;
    class UniformBspline;
    class MapManager;

    class MPCC
    {
    public:
        static constexpr int x_dim_ = 10;
        static constexpr int u_dim_ = 4;
        static constexpr int _dt_num = 1;
        static constexpr int _dt_den = 10;
        static constexpr int n_step_ = 10;
    
    private:
        // 环境信息
        std::shared_ptr<MapManager> map_manager_;

        // 无人机动力学模型
        std::shared_ptr<QuadrotorDynamic> quadrotor_dynamic_;

        // 参考轨迹
        std::shared_ptr<UniformBspline> ref_traj_;

        // 动态物体状态

        // 状态向量
        Eigen::VectorXd state_[n_step_];

        // 状态对控制量的雅可比矩阵
        Eigen::Matrix<double, x_dim_, n_step_ * u_dim_> state_g_[n_step_];

        // B样条的控制点
        Eigen::MatrixXd ctrl_pts_;
        Eigen::MatrixXd v_ctrl_pts_;
        Eigen::MatrixXd a_ctrl_pts_;

        // 样条曲线参数
        double ts_;
        double t_max_;

        // 上一次输出和初始状态
        Eigen::Matrix<double, u_dim_, 1> last_u_;
        Eigen::Matrix<double, x_dim_, 1> init_state_;

        // 1+1+1+4+2+1+1 = 11
        // 轮廓误差权重：0，滞后误差权重：1，进度权重：2，输入权重：3-6，yaw角权重：7-8，静态障碍物权重：9，动态障碍物权重：10
        Eigen::Matrix<double, 11, 1> cost_w_;
        const double dt_;

        // 非线性优化器
        nlopt::opt opt_;

        // // 在A*中寻找最近的点
        // Eigen::Vector3d searchMinDistPoint(const std::vector<Eigen::Vector3d> &path, const Eigen::Vector3d &pos);

    public:
        MPCC(std::shared_ptr<QuadrotorDynamic> &quadrotor_dynamic, 
             std::shared_ptr<UniformBspline> &ref_traj, 
             std::string algorithm = "LD_LBFGS", 
             int maxeval = 500);
        ~MPCC();
        
        // LD_AUGLAG LD_LBFGS
        static double cost_func(const std::vector<double> &u, std::vector<double> &grad, MPCC *instance);

        static void roll_constraint( unsigned m, double *result, unsigned n, const double *u, 
                                     double *gradient, /* NULL if not needed */
                                     MPCC *instance);
        
        static void pitch_constraint( unsigned m, double *result, unsigned n, const double *u,
                                      double *gradient, /* NULL if not needed */
                                      MPCC *instance);

        void set_w(Eigen::Matrix<double, 11, 1> &w);

        int solve(  const Eigen::Matrix<double, x_dim_, 1> &state,
                    const Eigen::MatrixXd &ctrl_pts,
                    const Eigen::MatrixXd &v_ctrl_pts,
                    const Eigen::MatrixXd &a_ctrl_pts,
                    const double ts,
                    const double len,
                    const Eigen::Matrix<double, u_dim_, 1> last_u,
                    Eigen::Matrix<double, n_step_, u_dim_> &predict_u, 
                    Eigen::Matrix<double, n_step_, x_dim_> &x_predict,
                    Eigen::Matrix<double, n_step_ + 1, 1> &t_index);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}

#endif