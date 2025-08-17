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
        static constexpr int dt_num_ = 1;
        static constexpr int dt_den_ = 10;
        static constexpr int n_step_ = 15;
    
    private:
        // 无人机动力学模型
        std::shared_ptr<QuadrotorDynamic> quadrotor_dynamic_;

        // 参考轨迹
        std::shared_ptr<UniformBspline> ref_traj_;

        // 动态物体状态

        int variable_num_{};

        // 状态向量
        Eigen::VectorXd state_[n_step_];

        // 状态对控制量的雅可比矩阵
        Eigen::Matrix<double, x_dim_, n_step_ * u_dim_> state_g_[n_step_];

        Eigen::Matrix<double, 3, 1> ref_pos_[n_step_ + 1];                      // 轨迹上的三维空间坐标
        Eigen::Matrix<double, 3, 1> ref_vel_[n_step_ + 1];
        Eigen::Matrix<double, 3, n_step_ + 1> ref_pos_g_[n_step_ + 1];          // 轨迹上的三维空间坐标的梯度
        Eigen::Matrix<double, 3, 1> tangent_[n_step_ + 1];
        Eigen::Matrix<double, 3, n_step_ + 1> tangent_g_[n_step_ + 1];

        Eigen::Matrix<double, 3, 1> err_[n_step_ + 1];
        Eigen::Matrix<double, 3, u_dim_ * n_step_ + n_step_ + 1> err_g_[n_step_ + 1];
        Eigen::Matrix<double, 3, 1> lag_err_[n_step_ + 1];
        Eigen::Matrix<double, 3, u_dim_ * n_step_ + n_step_ + 1> lag_err_g_[n_step_ + 1];
        Eigen::Matrix<double, 3, 1> contour_err_[n_step_ + 1];
        Eigen::Matrix<double, 3, u_dim_ * n_step_ + n_step_ + 1> contour_err_g_[n_step_ + 1];

        double state_yaw_[n_step_ + 1];
        Eigen::Vector4d state_yaw_g_[n_step_ + 1];

        std::vector<double> g_lag_, g_contour_, g_progress_, g_u_rate_, g_u_trust_, g_r_yaw_, g_v_yaw_, g_sfc_, g_ov_;


        // 上一次输出和初始状态
        Eigen::Matrix<double, u_dim_, 1> last_u_;
        Eigen::Matrix<double, x_dim_, 1> init_state_;
        std::vector<double> uv_, lb_, ub_;


        // 重要中间数据
        Eigen::Matrix<double, x_dim_, x_dim_> x1dotx0_;
        Eigen::Matrix<double, x_dim_, u_dim_> x1dotu_;

        // B样条的控制点
        Eigen::MatrixXd ctrl_pts_;
        Eigen::MatrixXd v_ctrl_pts_;

        // 样条曲线参数
        double ts_{};
        double t_max_{};

        std::vector<Eigen::MatrixXd> hPolys_;
        std::vector<int> path2hPloy_idx_;

        // 动态物体
        std::vector<Eigen::Matrix<double, 9, 1>> ob_dynamics_;

        // 1+1+1+2+2+1+1 = 9
        // 滞后误差权重：0，轮廓误差权重：1，进度误差权重：2，偏航角权重：3-4，控制输入权重：5-6，安全飞行走廊权重：7，速度障碍权重：8
        Eigen::Matrix<double, 9, 1> cost_w_;
        const double dt_;
        double dt_inv_;

        // 非线性优化器
        nlopt::opt opt_;

        // // 在A*中寻找最近的点
        // Eigen::Vector3d searchMinDistPoint(const std::vector<Eigen::Vector3d> &path, const Eigen::Vector3d &pos);

        void calcStateYaw();

        void calcState(const std::vector<double> &u);

        void calcRefTagAndError(const std::vector<double> &u);

        void calcState2hPloyIdx(const std::vector<double> &u, std::vector<int> &state2hPloy_idx);

        void combineCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient);

        void calcLagCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient);

        void calcContourCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient);

        void calcProgressCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient);

        void calcRYawCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient);

        void calcVYawCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient);

        void calcRateCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient);

        void calcTrustCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient);

        void calcSFCCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient);

        void calcOVCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient);

        static double costFunction(const std::vector<double> &u, std::vector<double> &gradient, MPCC *instance);

        bool isInSFC(std::vector<int> ploy_idx,  Eigen::Vector3d &cur_pos);

        bool getCorVel(Eigen::Vector2d &cor_vel,
                       Eigen::Vector2d &cur_pos,
                       Eigen::Vector2d &cur_vel,
                       Eigen::Vector2d &dyn_pos,
                       Eigen::Vector2d &dyn_vel,
                       double &circle_radius);

    public:
        MPCC(std::shared_ptr<QuadrotorDynamic> &quadrotor_dynamic,
             std::string algorithm = "LD_LBFGS", 
             int maxeval = 200,
             double maxtime = 0.015);

        ~MPCC();
        
        // LD_AUGLAG LD_LBFGS

        void set_w(Eigen::Matrix<double, 9, 1> &w);
        
        // int solver( const Eigen::Matrix<double, x_dim_, 1> &state,
        //             const Eigen::MatrixXd &ctrl_pts,
        //             const Eigen::MatrixXd &v_ctrl_pts,
        //             const double ts,
        //             const double tmax,
        //             const std::vector<Eigen::MatrixXd> &hPolys,
        //             const std::vector<int> path2hPloy_idx,
        //             const Eigen::Matrix<double, u_dim_, 1> &last_u,
        //             Eigen::Matrix<double, n_step_, u_dim_> &predict_u, 
        //             Eigen::Matrix<double, n_step_, x_dim_> &x_predict,
        //             Eigen::Matrix<double, n_step_ + 1, 1> &t_index);
        int solver( const Eigen::Matrix<double, x_dim_, 1> &state,
                    const Eigen::MatrixXd &ctrl_pts,
                    const Eigen::MatrixXd &v_ctrl_pts,
                    const double ts,
                    const double tmax,
                    const std::vector<Eigen::MatrixXd> &hPolys,
                    const std::vector<int> &path2hPloy_idx,
                    const std::vector<Eigen::Matrix<double, 9, 1>> &ob_dynamics,
                    const Eigen::Matrix<double, u_dim_, 1> &last_u,
                    Eigen::Matrix<double, n_step_, u_dim_> &predict_u, 
                    Eigen::Matrix<double, n_step_, x_dim_> &x_predict,
                    Eigen::Matrix<double, n_step_ + 1, 1> &t_index);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}

#endif