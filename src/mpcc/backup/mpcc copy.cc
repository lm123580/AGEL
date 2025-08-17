#include "mpcc.h"
#include "uniform_bspline.h"
#include "quadrotor_dynamic.h"

namespace AGEL
{
    MPCC::MPCC(std::shared_ptr<QuadrotorDynamic> &quadrotor_dynamic, 
               std::shared_ptr<UniformBspline> &ref_traj, 
               std::string algorithm, int maxeval):
               dt_((double)_dt_num / _dt_den),
               opt_(algorithm.c_str(), u_dim_ * n_step_ + n_step_ + 1)
    {
        quadrotor_dynamic_ = quadrotor_dynamic;

        ref_traj_ = ref_traj;

        for (int k = 0; k < n_step_; k++) 
        {
            state_[k].setZero();
            state_g_[k].setZero();
        }

        // std::vector<double> tols(n_step_, 1e-8);

        // 设置优化的成本函数
        opt_.set_min_objective((nlopt::vfunc)&MPCC::cost_func, this);
        // 设置不等式误差约束
        // opt_.add_inequality_mconstraint((nlopt::mfunc)&MPCC::roll_constraint, this, tols);
        // opt_.add_inequality_mconstraint((nlopt::mfunc)&MPCC::pitch_constraint, this, tols);
        // 优化变量的相对容差（优化的变量变化小于1e-16，认为优化量已经收敛）
        opt_.set_xtol_rel(1e-12);
        // 优化目标的相对容差（目标函数的值变化小于1e-8，认为优化量已经收敛）
        opt_.set_ftol_rel(1e-8);
        // 最大评估次数
        opt_.set_maxeval(maxeval);
        double max_time = 0.01;
        opt_.set_maxtime(max_time);

        // // 局部优化器
        // nlopt::opt local_opt("LD_LBFGS", u_dim_ * n_step_ + n_step_ + 1);
        // local_opt.set_xtol_rel(1e-16);
        // local_opt.set_ftol_rel(1e-12);
        // local_opt.set_maxeval(maxeval);
        // local_opt.set_vector_storage(30);
        // opt_.set_local_optimizer(local_opt);
    }


    MPCC::~MPCC() = default;


    void MPCC::set_w(Eigen::Matrix<double, 11, 1> &w) 
    {
        cost_w_ = w;
    }

    // void MPCC::roll_constraint( unsigned m, double *result, unsigned n, const double *u, 
    //                             double *gradient, /* NULL if not needed */
    //                             MPCC *instance)
    // {
    //     Eigen::VectorXd *state = instance->state_;
    //     Eigen::Matrix<double, x_dim_, n_step_ * u_dim_> *state_g = instance->state_g_;

    //     for (int k = 0; k < m; k++) 
    //     {
    //         double &qw = state[k][6];
    //         double &qx = state[k][7];
    //         double &qy = state[k][8];
    //         double &qz = state[k][9];
    //         double tmpx = 2. * (qw * qy - qx * qz);
    //         double tmpy = 1 - 2. * (qx * qx + qy * qy);
    //         double tmpnorm = pow(tmpx, 2) + pow(tmpy, 2);
    //         double roll = std::atan2(tmpx, tmpy);
    //         Eigen::Vector2d roll_g_tmp(tmpy / tmpnorm, -tmpx / tmpnorm);
    //         Eigen::Vector4d roll_g;
    //         roll_g[0] = roll_g_tmp.x() * 2 * qy;
    //         roll_g[1] = roll_g_tmp.x() * -2 * qz + roll_g_tmp.y() * -4 * qx;
    //         roll_g[2] = roll_g_tmp.x() * 2 * qw + roll_g_tmp.y() * -4 * qy;
    //         roll_g[3] = roll_g_tmp.x() * -2 * qx;
            
    //         result[k] = roll * roll - (M_PI_2) * (M_PI_2);
    //         if (gradient) 
    //         {
    //             for (int i = 0; i < n_step_ * u_dim_; i++) 
    //             {
    //                 if (roll > 0)
    //                 {
    //                     gradient[k * n + i] +=  2 * roll_g[0] * state_g[k](6, i) + 
    //                                             2 * roll_g[1] * state_g[k](7, i) +
    //                                             2 * roll_g[2] * state_g[k](8, i) + 
    //                                             2 * roll_g[3] * state_g[k](9, i);                        
    //                 }
    //                 else
    //                 {
    //                     gradient[k * n + i] -=  2 * roll_g[0] * state_g[k](6, i) + 
    //                                             2 * roll_g[1] * state_g[k](7, i) +
    //                                             2 * roll_g[2] * state_g[k](8, i) + 
    //                                             2 * roll_g[3] * state_g[k](9, i);   
    //                 }
    //             }
                
    //             for (int i = n_step_ * u_dim_; i < n; i++) 
    //             {
    //                 gradient[k * n + i] = 0;
    //             }
    //         }
    //     }
    // }


    // void MPCC::pitch_constraint(unsigned m, double *result, unsigned n, const double *u, 
    //                             double *gradient, /* NULL if not needed */
    //                             MPCC *instance)
    // {
    //     Eigen::VectorXd *state = instance->state_;
    //     Eigen::Matrix<double, x_dim_, n_step_ * u_dim_> *state_g = instance->state_g_;

    //     for (int k = 0; k < m; k++) 
    //     {
    //         double &qw = state[k][6];
    //         double &qx = state[k][7];
    //         double &qy = state[k][8];
    //         double &qz = state[k][9];
    //         double tmpx = 2. * (qx * qy + qw * qz);
    //         double tmpy = 1 - 2. * (qy * qy + qz * qz);
    //         double tmpnorm = pow(tmpx, 2) + pow(tmpy, 2);
    //         double pitch = std::atan2(tmpx, tmpy);
    //         Eigen::Vector2d pitch_g_tmp(tmpy / tmpnorm, -tmpx / tmpnorm);
    //         Eigen::Vector4d pitch_g;
    //         pitch_g[0] = pitch_g_tmp.x() * 2 * qy + pitch_g_tmp.y() * 2 * qw;
    //         pitch_g[1] = pitch_g_tmp.x() * 2 * qx + pitch_g_tmp.y() * -4 * qy;
    //         pitch_g[2] = pitch_g_tmp.x() * 2 * qz + pitch_g_tmp.y() * -4 * qz;
    //         pitch_g[3] = pitch_g_tmp.x() * -2 * qy;

    //         result[k] = pitch * pitch - (M_PI_4) * (M_PI_4);

    //         if (gradient) 
    //         {
    //             for (int i = 0; i < n_step_ * u_dim_; i++) 
    //             {
    //                 gradient[k * n + i] = 2 * pitch_g[0] * state_g[k](6, i) + 
    //                                       2 * pitch_g[1] * state_g[k](7, i) +
    //                                       2 * pitch_g[2] * state_g[k](8, i) + 
    //                                       2 * pitch_g[3] * state_g[k](9, i);
    //             }
                
    //             for (int i = n_step_ * u_dim_; i < n; i++) 
    //             {
    //                 gradient[k * n + i] = 0;
    //             }
    //         }
    //     }
    // }

    // Eigen::Vector3d MPCC::searchMinDistPoint(const std::vector<Eigen::Vector3d> &path, const Eigen::Vector3d &pos)
    // {
    //     if (path.empty()) 
    //     {
    //         std::cout << "WARN !!!!!!!!!" << std::endl;
    //         return Eigen::Vector3d(0, 0, 0);
    //     }

    //     Eigen::Vector3d min_point;
    //     double dist, min_dist = std::numeric_limits<double>::max();
    //     for (const auto &point : path) 
    //     {
    //         dist = (point - pos).squaredNorm();
    //         if (dist < min_dist) 
    //         {
    //             min_dist = dist;
    //             min_point = point;
    //         }
    //     }

    //     return min_point;
    // }


    double MPCC::cost_func(const std::vector<double> &u, std::vector<double> &grad, MPCC *instance)
    {
        // 静态地图
        // auto &map = instance->map_manager_->map_;

        // 系统状态及梯度
        Eigen::VectorXd *state = instance->state_;
        Eigen::Matrix<double, x_dim_, n_step_ * u_dim_> *state_g = instance->state_g_;

        // 样条参数
        // double ts = instance->ts_;
        double t_max = instance->t_max_;

        // 每步的时间间隔dt
        const double &dt = instance->dt_;

        // 权重
        Eigen::Matrix<double, 11, 1> &cost_w = instance->cost_w_;

        // 优化得到 u theta 和 1，1是初始点，时间是n+1
        Eigen::Matrix<double, u_dim_ * n_step_ + n_step_ + 1, 1> g;
        g.setZero();
        Eigen::Matrix<double, u_dim_, 1> uvec;

        // 根据无人机动力学，推导后n_steps步后，无人机的状态
        uvec << u[0], u[1], u[2], u[3];
        Eigen::Matrix<double, x_dim_, x_dim_> x1dotx0;
        Eigen::Matrix<double, x_dim_, u_dim_> x1dotu;
        instance->quadrotor_dynamic_->rk4_func(instance->init_state_, uvec, dt, state[0], x1dotx0, x1dotu);
        state_g[0].block(0, 0, x_dim_, u_dim_) = x1dotu;
        for (int k = 1; k < n_step_; k++) 
        {
            uvec << u[0 + k * u_dim_], u[1 + k * u_dim_], u[2 + k * u_dim_], u[3 + k * u_dim_];
            
            instance->quadrotor_dynamic_->rk4_func(state[k - 1], uvec, dt, state[k], x1dotx0, x1dotu);
            
            state_g[k].block(0, k * u_dim_, x_dim_, u_dim_) = x1dotu;
            state_g[k].block(0, 0, x_dim_, u_dim_ * k) = x1dotx0 * state_g[k - 1].block(0, 0, x_dim_, u_dim_ * k);
        }

        // 计算参考位姿（reference theta）和切向向量（tangent vector）
        Eigen::Matrix<double, 3, 1> ref_pos[n_step_ + 1];                  // 轨迹上的三维空间坐标
        Eigen::Matrix<double, 3, 1> ref_pos_tmp_g[n_step_ + 1];
        Eigen::Matrix<double, 3, n_step_ + 1> ref_pos_g[n_step_ + 1];      // 轨迹上的三维空间坐标的梯度
        Eigen::Matrix<double, 3, 1> tangent[n_step_ + 1];
        Eigen::Matrix<double, 3, n_step_ + 1> tangent_g[n_step_ + 1];
        double t_sum = u[u_dim_ * n_step_ + n_step_];                       // 初始时间
        
        for (int k = 0; k < n_step_ + 1; k++) 
        {
            bool over = false;
            if (t_sum >= t_max) 
            {
                t_sum = t_max;
                over = true;
            }
            Eigen::VectorXd ref_pos_k_g;
            ref_pos[k] = instance->ref_traj_->getBsplineValueFast(t_sum, instance->ctrl_pts_, 3, ref_pos_k_g);
            ref_pos_tmp_g[k] = ref_pos_k_g;
            Eigen::VectorXd tmp2;
            Eigen::Matrix<double, 3, 1> tmp = instance->ref_traj_->getBsplineValueFast(t_sum, instance->v_ctrl_pts_, 2, tmp2); // tmp是v，等同于ref_pos_k_g
            double snorm = tmp.squaredNorm();
            double norm = tmp.norm();
            tangent[k] = tmp / norm;    // 单位方向向量
            ref_pos_g[k].setZero();
            tangent_g[k].setZero();     // 方向的变化梯度
            if (!over) 
            {
                ref_pos_g[k].block(0, n_step_, 3, 1) = ref_pos_k_g;
                tangent_g[k].block(0, n_step_, 3, 1) = (tmp2 * norm - tmp * 0.5 * (1 / norm) * 2 * tmp.dot(tmp2)) / snorm;
                for (int i = 0; i < k; i++) 
                {
                    ref_pos_g[k].block(0, i, 3, 1) = ref_pos_k_g;
                    tangent_g[k].block(0, i, 3, 1) = (tmp2 * norm - tmp * 0.5 * (1 / norm) * 2 * tmp.dot(tmp2)) / snorm;
                }
            }
            if (k < n_step_) 
            {
                t_sum += u[u_dim_ * n_step_ + k];
            }
        }

        // 计算预测状态与参考状态的误差向量
        Eigen::Matrix<double, 3, 1> err[n_step_ + 1];
        Eigen::Matrix<double, 3, u_dim_ * n_step_ + n_step_ + 1> err_g[n_step_ + 1];
        for (int k = 0; k < n_step_ + 1; k++) {
            if (k > 0) 
            {
                err[k] = state[k - 1].block(0, 0, 3, 1) - ref_pos[k];
                err_g[k].setZero();
                err_g[k].block(0, u_dim_ * n_step_, 3, n_step_ + 1) = -ref_pos_g[k];                            // 时间项
                err_g[k].block(0, 0, 3, u_dim_ * n_step_) = state_g[k - 1].block(0, 0, 3, u_dim_ * n_step_);    // 状态项
            } 
            else 
            {
                err[k] = instance->init_state_.block(0, 0, 3, 1) - ref_pos[k];
                err_g[k].setZero();
                err_g[k].block(0, u_dim_ * n_step_, 3, n_step_ + 1) = -ref_pos_g[k];
            }
        }

        // 计算滞后误差
        Eigen::Matrix<double, 3, 1> lag_err[n_step_ + 1];
        Eigen::Matrix<double, 3, u_dim_ * n_step_ + n_step_ + 1> lag_err_g[n_step_ + 1];
        double lag_e[n_step_ + 1];
        Eigen::Matrix<double, u_dim_ * n_step_ + n_step_ + 1, 1> lag_e_g[n_step_ + 1];
        for (int k = 0; k < n_step_ + 1; k++) 
        {
            lag_e[k] = err[k].dot(tangent[k]);
            lag_e_g[k] = (tangent[k].transpose() * err_g[k]).transpose();
            lag_e_g[k].block(u_dim_ * n_step_, 0, n_step_ + 1, 1) += (err[k].transpose() * tangent_g[k]).transpose();
            lag_err[k] = lag_e[k] * tangent[k];
            lag_err_g[k] = tangent[k] * lag_e_g[k].transpose();
            lag_err_g[k].block(0, u_dim_ * n_step_, 3, n_step_ + 1) += lag_e[k] * tangent_g[k];
            
            lag_e_g[k] = 2 * lag_e[k] * lag_e_g[k];
            lag_e[k] = lag_e[k] * lag_e[k];
        }
        
        // 计算轮廓误差
        Eigen::Matrix<double, 3, 1> contour_err[n_step_ + 1];
        Eigen::Matrix<double, 3, u_dim_ * n_step_ + n_step_ + 1> contour_err_g[n_step_ + 1];
        double contour_e[n_step_ + 1];
        Eigen::Matrix<double, u_dim_ * n_step_ + n_step_ + 1, 1> contour_e_g[n_step_ + 1];
        for (int k = 0; k < n_step_ + 1; k++) 
        {
            contour_err[k] = err[k] - lag_err[k];
            contour_err_g[k] = err_g[k] - lag_err_g[k];
            contour_e[k] = contour_err[k].squaredNorm();
            contour_e_g[k] = (2 * contour_err[k].transpose() * contour_err_g[k]).transpose();
        }

        // 计算跟踪进度的成本
        double t_cost = 0.0, tmp_t;
        Eigen::Matrix<double, u_dim_ * n_step_ + n_step_ + 1, 1> t_cost_g;
        t_cost_g.setZero();
        tmp_t = u[u_dim_ * n_step_ + n_step_];
        for (int i = 0; i < n_step_; i++)
        {
            tmp_t += u[u_dim_ * n_step_ + i];
        }

        if (tmp_t < t_max)
        {
            t_cost_g[u_dim_ * n_step_ + n_step_] = -1;
            for (int k = 0; k < n_step_; k++) 
            {
                t_cost -= u[u_dim_ * n_step_ + k];
                t_cost_g[u_dim_ * n_step_ + k] = -1;
            }            
        }

        // 计算静态障碍物的成本
        // Eigen::Vector3d state_pos_tmp;
        // for (int k = 0; k < n_step_; k++)
        // {
        //     state_pos_tmp = state[k].block(0, 0, 3, 1);
        //     if (map->getInflateOccupancy(state_pos_tmp))
        //     {
        //         cost_w[0] = 10.0;
        //         cost_w[1] = 20.0;
        //         std::cout << k << ": " << state_pos_tmp << std::endl;
        //         // cost_w[1] = 80.0;
        //         // cost_w[7] = 2.0;
        //         break ;
        //     }
        // }

        double static_cost = 0.0;
        // bool static_flag = false;
        std::vector<std::pair<int, Eigen::Vector3d>> pos_in_occupancy;
        Eigen::Matrix<double, u_dim_ * n_step_ + n_step_ + 1, 1> static_cost_g;
        Eigen::Vector3d static_g, static_pos;
        static_cost_g.setZero();

        
        // for (int k = 0; k < n_step_; k++)                   // 检测预测位置
        // {
        //     static_pos = state[k].block(0, 0, 3, 1);
        //     if (map->getInflateOccupancy(static_pos))
        //     {
        //         pos_in_occupancy.push_back(std::make_pair(k, static_pos));
        //     }
        // }

        // int pos_in_occupancy_size = pos_in_occupancy.size();
        // if (pos_in_occupancy_size > 0)                    // 如果在静态障碍物中
        // {
        //     // A*路径
        //     auto &path = instance->astar_path_;
        //     static_flag = true;
        //     Eigen::Vector3d temp_point;
        //     for (auto &pos_pair: pos_in_occupancy)
        //     {
        //         int k = pos_pair.first;
        //         temp_point = instance->searchMinDistPoint(path, pos_pair.second);
        //         static_g = temp_point - pos_pair.second;
        //         double tmp_dis = static_g.norm();
        //         static_cost += tmp_dis * tmp_dis;
        //         static_cost_g.block(0, 0, u_dim_ * n_step_, 1) += (2 * tmp_dis * static_g.transpose() * state_g[k].block(0, 0, 3, u_dim_ * n_step_)).transpose();
        //     }
        // }

        // c_p << -12, 0, 1;
        // for (int k = 0; k < n_step_; k++) 
        // {
        //     c_pos = state[k].block(0, 0, 3, 1);
        //     c_pos[2] = 1.0;
        //     c_v = state[k].block(3, 0, 3, 1);
        //     c_v[2] = 0.0;
        //     c_g = c_p - c_pos;
        //     dis = c_g.norm();
        //     if (dis < 1.5) 
        //     {
        //         c_g = (c_g.normalized() - c_g.dot(c_v) * c_v.normalized() * 0.5).normalized();
        //         double tmp = 2.0 - dis;
        //         double tmp2 = pow(tmp, 2);
        //         c_cost += tmp2;
        //         c_cost_g.block(0, 0, u_dim_ * n_step_, 1) += (2 * tmp * c_g.transpose() * state_g[k].block(0, 0, 3, u_dim_ * n_step_)).transpose();            
        //     }

        //     if (dis < 1.75)
        //     {
        //         c_flag = true;
        //     }
        // }               

        // 计算两个偏航角成本，即偏航角与参考轨迹的夹角和偏航角与速度的夹角
        double r_yawcost = 0.0, v_yawcost = 0.0;
        Eigen::Matrix<double, u_dim_ * n_step_ + n_step_ + 1, 1> r_yawcost_g, v_yawcost_g;
        v_yawcost_g.setZero(), r_yawcost_g.setZero();
        for (int k = 0; k < n_step_; k++) 
        {
            double &rx = ref_pos_tmp_g[k+1][0];
            double &ry = ref_pos_tmp_g[k+1][1];
            double &vx = state[k][3];
            double &vy = state[k][4];
            double &qw = state[k][6];
            double &qx = state[k][7];
            double &qy = state[k][8];
            double &qz = state[k][9];
            double tmpx = 1. - 2. * (qy * qy + qz * qz);
            double tmpy = 2. * (qw * qz + qx * qy);
            double tmpnorm = pow(tmpx, 2) + pow(tmpy, 2);
            double yaw = std::atan2(tmpy, tmpx);
            Eigen::Vector2d yaw_g_tmp(-tmpy / tmpnorm, tmpx / tmpnorm);
            Eigen::Vector4d yaw_g;
            yaw_g.setZero();
            yaw_g(0) = yaw_g_tmp.y() * 2 * qz;
            yaw_g(1) = yaw_g_tmp.y() * 2 * qy;
            yaw_g(2) = yaw_g_tmp.x() * (-4 * qy) + yaw_g_tmp.y() * 2 * qx;
            yaw_g(3) = yaw_g_tmp.x() * (-4 * qz) + yaw_g_tmp.y() * 2 * qw;
            
            // if (pow(vx, 2) + pow(vy, 2) < 0.25)
            // {
            // 计算偏航角1成本，即偏航角与参考轨迹的夹角
            double rang = std::atan2(ry, rx);
            double rxynorm = pow(rx, 2) + pow(ry, 2);
            Eigen::Vector2d rang_g(-ry / rxynorm, rx / rxynorm);
            if (yaw - rang > M_PI) 
            {
                r_yawcost += 2 * M_PI - (yaw - rang);
                r_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                    (-yaw_g.transpose() * state_g[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (rang_g.transpose() * state_g[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            } 
            else if (rang - yaw > M_PI) 
            {
                r_yawcost += 2 * M_PI + (yaw - rang);
                r_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                    (yaw_g.transpose() * state_g[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (-rang_g.transpose() * state_g[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            } 
            else if (yaw > rang) 
            {
                r_yawcost += yaw - rang; 
                r_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                    (yaw_g.transpose() * state_g[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (-rang_g.transpose() * state_g[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            } 
            else 
            {
                r_yawcost += -yaw + rang;
                r_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                    (-yaw_g.transpose() * state_g[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (rang_g.transpose() * state_g[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            }
            // }
            // else
            // {
            // 计算偏航角2成本，即偏航角与速度的夹角
            double vang = std::atan2(vy, vx);
            double vxynorm = pow(vx, 2) + pow(vy, 2);
            Eigen::Vector2d vang_g(-vy / vxynorm, vx / vxynorm);
            if (yaw - vang > M_PI) 
            {
                v_yawcost += 2 * M_PI - (yaw - vang);
                v_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                        (-yaw_g.transpose() * state_g[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (vang_g.transpose() * state_g[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            } 
            else if (vang - yaw > M_PI) 
            {
                v_yawcost += 2 * M_PI + (yaw - vang);
                v_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                        (yaw_g.transpose() * state_g[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (-vang_g.transpose() * state_g[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            } 
            else if (yaw > vang) 
            {
                v_yawcost += yaw - vang; 
                v_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                        (yaw_g.transpose() * state_g[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (-vang_g.transpose() * state_g[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            } 
            else 
            {
                v_yawcost += -yaw + vang;
                v_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                        (-yaw_g.transpose() * state_g[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (vang_g.transpose() * state_g[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            }
            // }
        }

        // if (static_flag)
        // {
        //     cost_w[0] *= 1.0;
        //     // cost_w[1] *= 10;
        //     // cost_w[2] *= 0.1;
        //     // cost_w[3] *= 2.5;
        //     // cost_w[4] *= 2.5;
        //     // cost_w[5] *= 2.5;
        //     // cost_w[6] *= 2.5;
        // }
        
        // 计算控制输入的成本
        double ucost = 0.0;
        for (int k = 0; k < n_step_; k++) 
        {
            int of = k * u_dim_;
            // past_u是指上一步的控制输入
            Eigen::Matrix<double, u_dim_, 1> past_u;
            if (k == 0) 
            {
                past_u = instance->last_u_;
            } 
            else 
            {
                past_u = Eigen::Matrix<double, u_dim_, 1>(
                    u[of - 4], u[of - 3], u[of - 2], u[of - 1]
                );
            }
            ucost += cost_w[3] * pow(u[of + 0] - past_u(0), 2)
                  +  cost_w[4] * pow(u[of + 1] - past_u(1), 2)
                  +  cost_w[5] * pow(u[of + 2] - past_u(2), 2)
                  +  cost_w[6] * pow(u[of + 3] - past_u(3), 2);
            g[of + 0] += 2 * cost_w[3] * (u[of + 0] - past_u(0));
            g[of + 1] += 2 * cost_w[4] * (u[of + 1] - past_u(1));
            g[of + 2] += 2 * cost_w[5] * (u[of + 2] - past_u(2));
            g[of + 3] += 2 * cost_w[6] * (u[of + 3] - past_u(3));
            
            if (k != 0) 
            {
                g[of - 4] -= 2 * cost_w[3] * (u[of + 0] - past_u(0));
                g[of - 3] -= 2 * cost_w[4] * (u[of + 1] - past_u(1));
                g[of - 2] -= 2 * cost_w[5] * (u[of + 2] - past_u(2));
                g[of - 1] -= 2 * cost_w[6] * (u[of + 3] - past_u(3));
            }
        }

        // 所有成本相加
        double cost = cost_w[2] * t_cost + ucost + cost_w[7] * r_yawcost + cost_w[8] * v_yawcost + 0 * static_cost;
        double lag_cost = 0.0;
        double contour_cost = 0.0;

        for (int k = 0; k < n_step_ + 1; k++) 
        {
            lag_cost += lag_e[k];
            contour_cost += contour_e[k];
            if (k == n_step_) 
            {
                cost += 2.5 * cost_w[0] * contour_e[k];
                g += 2.5 * cost_w[0] * contour_e_g[k];
                cost += 2.5 * cost_w[1] * lag_e[k];
                g += 2.5 * cost_w[1] * lag_e_g[k];
            } 
            else 
            {
                cost += cost_w[0] * contour_e[k];
                g += cost_w[0] * contour_e_g[k];
                cost += cost_w[1] * lag_e[k];
                g += cost_w[1] * lag_e_g[k];
            }
        }
        // std::cout << " ALL COST: " << cost << std::endl;
        // std::cout << " contour_cost: " << contour_cost << " lag_cost: " << lag_cost << " t_cost: " << cost_w[2] * t_cost << std::endl;
        // std::cout<< " ucost: " << ucost << " r_yawcost: " << cost_w[7] * r_yawcost << " v_yawcost: " << cost_w[8] * v_yawcost  << std::endl;

        // std::cout << "U IS :" << std::endl;
        // for (int i = 0; i < (int)u.size(); i++)
        // {
        //     if (i % 4 == 0)
        //         std::cout << std::endl;
        //     std::cout << u[i] << " ";
        // }
        // std::cout << std::endl;
        
        g += cost_w[2] * t_cost_g + cost_w[7] * r_yawcost_g + cost_w[8] * v_yawcost_g + 0 * static_cost_g;
        
        for (int i = 0; i < (int)grad.size(); i++) 
        {
            grad[i] = g(i);
        }

        return cost;
    }

    int MPCC::solve(
        const Eigen::Matrix<double, x_dim_, 1> &state,
        const Eigen::MatrixXd &ctrl_pts,
        const Eigen::MatrixXd &v_ctrl_pts,
        const Eigen::MatrixXd &a_ctrl_pts,
        const double ts,
        const double len,
        const Eigen::Matrix<double, u_dim_, 1> last_u,
        Eigen::Matrix<double, n_step_, u_dim_> &predict_u, 
        Eigen::Matrix<double, n_step_, x_dim_> &x_predict,
        Eigen::Matrix<double, n_step_ + 1, 1> &t_index) 
    {
        const double t_max = ctrl_pts.rows() * ts - 3 * ts;
        std::vector<double> uv(u_dim_ * n_step_ + n_step_ + 1);
        
        // 初始解
        for (int k = 0; k < n_step_; k++) 
        {
            uv[k * u_dim_ + 0] = last_u(0);
            uv[k * u_dim_ + 1] = last_u(1);
            uv[k * u_dim_ + 2] = last_u(2);
            uv[k * u_dim_ + 3] = last_u(3);
        }
        for (int k = 0; k < n_step_; k++) 
        {
            uv[n_step_ * u_dim_ + k] = t_index[k + 1] - t_index[k];
        }
        uv[n_step_ * u_dim_ + n_step_] = t_index[0];

        // 优化变量的边界
        std::vector<double> lb(uv.size()), ub(uv.size());
        // 控制的边界
        for (int k = 0; k < n_step_; k++) 
        {
            lb[k * u_dim_ + 0] = -M_PI_2;
            lb[k * u_dim_ + 1] = -M_PI_2;
            lb[k * u_dim_ + 2] = -M_PI_2;
            lb[k * u_dim_ + 3] = 0.0;

            ub[k * u_dim_ + 0] = M_PI_2;
            ub[k * u_dim_ + 1] = M_PI_2;
            ub[k * u_dim_ + 2] = M_PI_2;
            ub[k * u_dim_ + 3] = 0.9;
        }
        // 进度theta/t的边界
        for (int k = 0; k < n_step_; k++) 
        {
            lb[n_step_ * u_dim_ + k] = 0.005;
            ub[n_step_ * u_dim_ + k] = 0.50;
        }
        lb[n_step_ * u_dim_ + n_step_] = 0.0;
        ub[n_step_ * u_dim_ + n_step_] = t_max;
        
        opt_.set_lower_bounds(lb);
        opt_.set_upper_bounds(ub);

        // 限制初始控制量的范围
        for (int i = 0; i < (int)uv.size(); i++)
        {
            if (uv[i] > ub[i]) 
            {
                uv[i] = ub[i];
            } 
            else if (uv[i] < lb[i]) 
            {
                uv[i] = lb[i];
            }
        }

        // 参数初始化为0
        for (int k = 0; k < n_step_; k++) 
        {
            state_[k].setZero();
            state_g_[k].setZero();
        }
        
        init_state_ = state;
        ctrl_pts_ = ctrl_pts;
        v_ctrl_pts_ = v_ctrl_pts;
        a_ctrl_pts_ = a_ctrl_pts;
        ts_ = ts;
        t_max_ = t_max;
        last_u_ = last_u;

        double minf;
        try
        {
            // 解决优化问题
            opt_.optimize(uv, minf);

            // 从优化变量中获取控制输入
            for (int k = 0; k < n_step_; k++) 
            {
                predict_u(k, 0) = uv[k * u_dim_ + 0];
                predict_u(k, 1) = uv[k * u_dim_ + 1];
                predict_u(k, 2) = uv[k * u_dim_ + 2];
                predict_u(k, 3) = uv[k * u_dim_ + 3];
            }
            
            // 从优化变量中获取状态
            for (int k = 0; k < n_step_; k++) 
            {
                x_predict.row(k) = state_[k];
            }

            // 从优化变量中获取进度
            t_index[0] = uv[u_dim_ * n_step_ + n_step_];
            for (int k = 0; k < n_step_; k++) {
                t_index[k + 1] = t_index[k] + uv[u_dim_ * n_step_ + k];
            }

            return EXIT_SUCCESS;
        } 
        catch(std::exception &e) 
        {
            // 从优化变量中获取控制输入
            for (int k = 0; k < n_step_; k++) 
            {
                predict_u(k, 0) = uv[k * u_dim_ + 0];
                predict_u(k, 1) = uv[k * u_dim_ + 1];
                predict_u(k, 2) = uv[k * u_dim_ + 2];
                predict_u(k, 3) = uv[k * u_dim_ + 3];
            }

            // 从优化变量中获取状态
            for (int k = 0; k < n_step_; k++) 
            {
                x_predict.row(k) = state_[k];
            }
            
            // 从优化变量中获取进度
            t_index[0] = uv[u_dim_ * n_step_ + n_step_];
            for (int k = 0; k < n_step_; k++) 
            {
                t_index[k + 1] = t_index[k] + uv[u_dim_ * n_step_ + k];
            }

            return EXIT_FAILURE;
        }
    }
}
