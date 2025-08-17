#include "mpcc.h"
#include "uniform_bspline.h"
#include "quadrotor_dynamic.h"

namespace AGEL
{
    MPCC::MPCC(std::shared_ptr<QuadrotorDynamic> &quadrotor_dynamic, 
               std::string algorithm, int maxeval, double maxtime):
               dt_((double)dt_num_ / dt_den_),
               opt_(algorithm.c_str(), u_dim_ * n_step_ + n_step_ + 1)
    {
        dt_inv_ = 1.0 / dt_;
        quadrotor_dynamic_ = quadrotor_dynamic;

        ref_traj_.reset(new UniformBspline);

        // 初始化状态
        for (int k = 0; k < n_step_; k++) 
        {
            state_[k].resize(x_dim_);
            state_[k].setZero();
            state_g_[k].setZero();
        }

        variable_num_ = u_dim_ * n_step_ + n_step_ + 1;

        uv_.resize(variable_num_);
        lb_.resize(variable_num_);
        ub_.resize(variable_num_);

        // 控制的边界
        for (int k = 0; k < n_step_; k++) 
        {
            lb_[k * u_dim_ + 0] = -M_PI_2;
            lb_[k * u_dim_ + 1] = -M_PI_2;
            lb_[k * u_dim_ + 2] = -M_PI_2;
            lb_[k * u_dim_ + 3] = 0.00;

            ub_[k * u_dim_ + 0] = M_PI_2;
            ub_[k * u_dim_ + 1] = M_PI_2;
            ub_[k * u_dim_ + 2] = M_PI_2;
            ub_[k * u_dim_ + 3] = 0.95;
        }
        // 进度theta/t的边界
        for (int k = 0; k < n_step_; k++) 
        {
            lb_[n_step_ * u_dim_ + k] = 0.001;
            ub_[n_step_ * u_dim_ + k] = 0.750;
        }
        lb_[n_step_ * u_dim_ + n_step_] = 0.0;
        ub_[n_step_ * u_dim_ + n_step_] = 0.0;

        // 设置优化的成本函数
        opt_.set_min_objective((nlopt::vfunc)&MPCC::costFunction, this);
        // 优化变量的相对容差（优化的变量变化小于1e-16，认为优化量已经收敛）
        opt_.set_xtol_rel(1e-16);
        // 优化目标的相对容差（目标函数的值变化小于1e-12，认为优化量已经收敛）
        opt_.set_ftol_rel(1e-12);
        // 最大评估次数
        opt_.set_maxeval(maxeval);
        // 最大评估时间
        opt_.set_maxtime(maxtime);
    }


    MPCC::~MPCC() = default;


    void MPCC::set_w(Eigen::Matrix<double, 9, 1> &w) 
    {
        cost_w_ = w;
    }


    // 根据无人机动力学，推导后n_steps步后，无人机的状态
    void MPCC::calcState(const std::vector<double> &u)
    {
        Eigen::Matrix<double, u_dim_, 1> uvec;

        uvec << u[0], u[1], u[2], u[3];
        quadrotor_dynamic_->rk4_func(init_state_, uvec, dt_, state_[0], x1dotx0_, x1dotu_);
        state_g_[0].block(0, 0, x_dim_, u_dim_) = x1dotu_;
        for (int k = 1; k < n_step_; k++) 
        {
            uvec << u[0 + k * u_dim_], u[1 + k * u_dim_], u[2 + k * u_dim_], u[3 + k * u_dim_];
            
            quadrotor_dynamic_->rk4_func(state_[k - 1], uvec, dt_, state_[k], x1dotx0_, x1dotu_);
            
            state_g_[k].block(0, k * u_dim_, x_dim_, u_dim_) = x1dotu_;
            state_g_[k].block(0, 0, x_dim_, u_dim_ * k) = x1dotx0_ * state_g_[k - 1].block(0, 0, x_dim_, u_dim_ * k);
        }
    }


    // 根据状态计算偏航角
    void MPCC::calcStateYaw()
    {
        for (int k = 0; k < n_step_; k++) 
        {
            double &qw = state_[k][6];
            double &qx = state_[k][7];
            double &qy = state_[k][8];
            double &qz = state_[k][9];
            double tmpx = 1.0 - 2.0 * (qy * qy + qz * qz);
            double tmpy = 2.0 * (qw * qz + qx * qy);
            double tmpnorm = tmpx * tmpx + tmpy * tmpy;
            state_yaw_[k] = std::atan2(tmpy, tmpx);
            Eigen::Vector2d yaw_g_tmp(-tmpy / tmpnorm, tmpx / tmpnorm);
            state_yaw_g_[k](0) = yaw_g_tmp.y() * 2 * qz;
            state_yaw_g_[k](1) = yaw_g_tmp.y() * 2 * qy;
            state_yaw_g_[k](2) = yaw_g_tmp.x() * (-4 * qy) + yaw_g_tmp.y() * 2 * qx;
            state_yaw_g_[k](3) = yaw_g_tmp.x() * (-4 * qz) + yaw_g_tmp.y() * 2 * qw;
        }
    }


    void MPCC::calcRefTagAndError(const std::vector<double> &u)
    {
        bool isOver;
        double snorm, norm;
        double t_cur = u[u_dim_ * n_step_ + n_step_];                       // 初始时间
        
        Eigen::VectorXd ref_vel, ref_acc;

        int ref_size = n_step_ + 1;
        for (int k = 0; k < ref_size; k++) 
        {
            isOver = false;
            if (t_cur >= t_max_) 
            {
                t_cur = t_max_;
                isOver = true;
            }
            ref_pos_[k] = ref_traj_->getBsplineValueFast(t_cur, ctrl_pts_, 3, ref_vel);
            ref_vel_[k] = ref_vel;
            ref_traj_->getBsplineValueFast(t_cur, v_ctrl_pts_, 2, ref_acc);
            
            snorm = ref_vel.squaredNorm();
            norm = std::sqrt(snorm);
            
            tangent_[k] = ref_vel / norm;    // 单位方向向量
            ref_pos_g_[k].setZero();
            tangent_g_[k].setZero();         // 方向的变化梯度

            if (!isOver) 
            {
                ref_pos_g_[k].block(0, n_step_, 3, 1) = ref_vel;
                tangent_g_[k].block(0, n_step_, 3, 1) = (ref_acc * norm - ref_vel * 0.5 * (1 / norm) * 2 * ref_vel.dot(ref_acc)) / snorm;
                for (int i = 0; i < k; i++) 
                {
                    ref_pos_g_[k].block(0, i, 3, 1) = ref_vel;
                    tangent_g_[k].block(0, i, 3, 1) = (ref_acc * norm - ref_vel * 0.5 * (1 / norm) * 2 * ref_vel.dot(ref_acc)) / snorm;
                }
            }
            
            if (k < n_step_) 
            {
                t_cur += u[u_dim_ * n_step_ + k];
            }
        }

        for (int k = 0; k < ref_size; k++) 
        {
            err_g_[k].setZero();

            if (k > 0) 
            {
                err_[k] = state_[k - 1].block(0, 0, 3, 1) - ref_pos_[k];
                err_g_[k].block(0, u_dim_ * n_step_, 3, n_step_ + 1) = -ref_pos_g_[k];                            // 时间项
                err_g_[k].block(0, 0, 3, u_dim_ * n_step_) = state_g_[k - 1].block(0, 0, 3, u_dim_ * n_step_);    // 状态项
            } 
            else 
            {
                err_[k] = init_state_.block(0, 0, 3, 1) - ref_pos_[k];
                err_g_[k].block(0, u_dim_ * n_step_, 3, n_step_ + 1) = -ref_pos_g_[k];
            }
        }
    }


    // 计算当前状态对应多面体的索引
    void MPCC::calcState2hPloyIdx(const std::vector<double> &u, std::vector<int> &state2hPloy_idx)
    {
        state2hPloy_idx.clear();

        int t2path_idx;
        double t_cur = u[u_dim_ * n_step_ + n_step_];
        for (int k = 0; k < n_step_; k++) 
        {
            t_cur += u[u_dim_ * n_step_ + k];

            if (t_cur >= t_max_) 
            {
                t_cur = t_max_;
            }

            t2path_idx = std::floor(t_cur / ts_);
            state2hPloy_idx.emplace_back(path2hPloy_idx_[t2path_idx]);
        }
    }


    // 计算滞后误差
    void MPCC::calcLagCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient)
    {
        cost = 0.0;
        gradient.resize(variable_num_);

        int ref_size = n_step_ + 1;
        double lag_e[ref_size];
        Eigen::Matrix<double, u_dim_ * n_step_ + n_step_ + 1, 1> lag_e_g[ref_size], tmp_g;
        tmp_g.setZero();
        for (int k = 0; k < ref_size; k++) 
        {
            lag_e[k] = err_[k].dot(tangent_[k]);

            lag_e_g[k] = (tangent_[k].transpose() * err_g_[k]).transpose();
            lag_e_g[k].block(u_dim_ * n_step_, 0, n_step_ + 1, 1) += (err_[k].transpose() * tangent_g_[k]).transpose();
            
            lag_err_[k] = lag_e[k] * tangent_[k];
            lag_err_g_[k] = tangent_[k] * lag_e_g[k].transpose();
            lag_err_g_[k].block(0, u_dim_ * n_step_, 3, n_step_ + 1) += lag_e[k] * tangent_g_[k];
            
            lag_e_g[k] = 2 * lag_e[k] * lag_e_g[k];
            lag_e[k] = lag_e[k] * lag_e[k];
        }

        double weight = 1.00;
        for (int k = 0; k < ref_size - 1; k++)
        {
            cost += weight * lag_e[k];
            tmp_g += weight * lag_e_g[k];
            weight = weight * 1.00;
        }
        cost += weight * lag_e[n_step_];
        tmp_g += weight * lag_e_g[n_step_];
        // cost += 2.5 * lag_e[n_step_];
        // tmp_g += 2.5 * lag_e_g[n_step_];
        
        for (int k = 0; k < variable_num_; k++)
        {
            gradient[k] = tmp_g[k];
        }
    }


    // 计算轮廓误差
    void MPCC::calcContourCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient)
    {
        cost = 0.0;
        gradient.resize(variable_num_);

        int ref_size = n_step_ + 1;
        double contour_e[ref_size];
        Eigen::Matrix<double, u_dim_ * n_step_ + n_step_ + 1, 1> contour_e_g[ref_size], tmp_g;
        
        for (int k = 0; k < ref_size; k++) 
        {
            contour_err_[k] = err_[k] - lag_err_[k];
            contour_err_g_[k] = err_g_[k] - lag_err_g_[k];
            contour_e[k] = contour_err_[k].squaredNorm();
            contour_e_g[k] = (2 * contour_err_[k].transpose() * contour_err_g_[k]).transpose();
        }

        double weight = 1.00;
        for (int k = 0; k < ref_size - 1; k++)
        {
            cost += weight * contour_e[k];
            tmp_g += weight * contour_e_g[k];

            weight *= 1.00;
        }
        cost += weight * contour_e[n_step_];
        tmp_g += weight * contour_e_g[n_step_];
        // cost += 2.5 * contour_e[n_step_];
        // tmp_g += 2.5 * contour_e_g[n_step_];

        for (int k = 0; k < variable_num_; k++)
        {
            gradient[k] = tmp_g[k];
        }
    }


    // 计算跟踪进度成本
    void MPCC::calcProgressCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient)
    {
        cost = 0.0;
        gradient.resize(variable_num_);

        int t_start_idx = u_dim_ * n_step_;
        for (int k = 0; k < n_step_; k++)
        {
            cost -= u[t_start_idx + k];
            gradient[t_start_idx + k] = -1;
        }
        gradient[t_start_idx + n_step_] = -1;
    }


    // 计算与参考曲线的偏航角成本
    void MPCC::calcRYawCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient)
    {
        cost = 0.0;
        gradient.resize(variable_num_);

        double r_yawcost = 0.0;
        Eigen::Matrix<double, u_dim_ * n_step_ + n_step_ + 1, 1> r_yawcost_g, tmp_g;
        r_yawcost_g.setZero();
        for (int k = 0; k < n_step_; k++) 
        {
            double &yaw = state_yaw_[k];
            Eigen::Vector4d &yaw_g = state_yaw_g_[k];
            double &rx = ref_vel_[k][0];
            double &ry = ref_vel_[k][1];
            double rang = std::atan2(ry, rx);
            double rxynorm = rx * rx + ry * ry;
            Eigen::Vector2d rang_g(-ry / rxynorm, rx / rxynorm);
            
            if (yaw - rang > M_PI) 
            {
                r_yawcost += 2 * M_PI - (yaw - rang);
                r_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                    (-yaw_g.transpose() * state_g_[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (rang_g.transpose() * state_g_[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            } 
            else if (rang - yaw > M_PI) 
            {
                r_yawcost += 2 * M_PI + (yaw - rang);
                r_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                    (yaw_g.transpose() * state_g_[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (-rang_g.transpose() * state_g_[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            } 
            else if (yaw > rang) 
            {
                r_yawcost += yaw - rang; 
                r_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                    (yaw_g.transpose() * state_g_[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (-rang_g.transpose() * state_g_[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            } 
            else 
            {
                r_yawcost += -yaw + rang;
                r_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                    (-yaw_g.transpose() * state_g_[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (rang_g.transpose() * state_g_[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            }
        }

        cost = r_yawcost;
        for (int k = 0; k < variable_num_; k++)
        {
            gradient[k] = r_yawcost_g[k];
        }
    }


    // 计算速度的偏航角成本
    void MPCC::calcVYawCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient)
    {
        cost = 0.0;
        gradient.resize(variable_num_);

        double v_yawcost = 0.0;
        Eigen::Matrix<double, u_dim_ * n_step_ + n_step_ + 1, 1> v_yawcost_g;
        v_yawcost_g.setZero();
        for (int k = 0; k < n_step_; k++) 
        {
            double &yaw = state_yaw_[k];
            Eigen::Vector4d &yaw_g = state_yaw_g_[k];
            double &vx = state_[k][3];
            double &vy = state_[k][4];
            double vang = std::atan2(vy, vx);
            double vxynorm = pow(vx, 2) + pow(vy, 2);
            Eigen::Vector2d vang_g(-vy / vxynorm, vx / vxynorm);
            if (yaw - vang > M_PI) 
            {
                v_yawcost += 2 * M_PI - (yaw - vang);
                v_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                        (-yaw_g.transpose() * state_g_[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (vang_g.transpose() * state_g_[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            } 
            else if (vang - yaw > M_PI) 
            {
                v_yawcost += 2 * M_PI + (yaw - vang);
                v_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                        (yaw_g.transpose() * state_g_[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (-vang_g.transpose() * state_g_[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            } 
            else if (yaw > vang) 
            {
                v_yawcost += yaw - vang; 
                v_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                        (yaw_g.transpose() * state_g_[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (-vang_g.transpose() * state_g_[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            } 
            else 
            {
                v_yawcost += -yaw + vang;
                v_yawcost_g.block(0, 0, u_dim_ * n_step_, 1) += 
                        (-yaw_g.transpose() * state_g_[k].block(6, 0, 4, u_dim_ * n_step_)).transpose()
                    + (vang_g.transpose() * state_g_[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
            }
        }

        cost = v_yawcost;
        for (int k = 0; k < variable_num_; k++)
        {
            gradient[k] = v_yawcost_g[k];
        }
    }


    // 计算控制输入中速率变化的成本
    void MPCC::calcRateCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient)
    {
        cost = 0.0;
        gradient.resize(variable_num_);

        double rate_cost = 0.0;
        Eigen::Matrix<double, u_dim_, 1> past_u;
        Eigen::Matrix<double, u_dim_ * n_step_ + n_step_ + 1, 1> u_rate_g;
        u_rate_g.setZero();
        for (int k = 0; k < n_step_; k++) 
        {
            int of = k * u_dim_;

            if (k == 0) 
            {
                past_u = last_u_;
            } 
            else 
            {
                past_u = Eigen::Matrix<double, u_dim_, 1>(
                    u[of - 4], u[of - 3], u[of - 2], u[of - 1]
                );
            }
            rate_cost += pow(u[of + 0] - past_u(0), 2)
                      +  pow(u[of + 1] - past_u(1), 2)
                      +  pow(u[of + 2] - past_u(2), 2);
            u_rate_g[of + 0] += 2 * (u[of + 0] - past_u(0));
            u_rate_g[of + 1] += 2 * (u[of + 1] - past_u(1));
            u_rate_g[of + 2] += 2 * (u[of + 2] - past_u(2));
            
            if (k != 0) 
            {
                u_rate_g[of - 4] -= 2 * (u[of + 0] - past_u(0));
                u_rate_g[of - 3] -= 2 * (u[of + 1] - past_u(1));
                u_rate_g[of - 2] -= 2 * (u[of + 2] - past_u(2));
            }
        }

        cost = rate_cost;
        for (int k = 0; k < variable_num_; k++)
        {
            gradient[k] = u_rate_g[k];
        }
    }


    // 计算控制输入中推力变化的成本
    void MPCC::calcTrustCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient)
    {
        cost = 0.0;
        gradient.resize(variable_num_);

        double trust_cost = 0.0;
        double last_trust;
        Eigen::Matrix<double, u_dim_ * n_step_ + n_step_ + 1, 1> u_trust_g;
        u_trust_g.setZero();
        for (int k = 0; k < n_step_; k++) 
        {
            int of = k * u_dim_;

            if (k == 0) 
            {
                last_trust = last_u_[3];
            } 
            else 
            {
                last_trust = u[of - 1];
            }
            trust_cost += pow(u[of + 3] - last_trust, 2);
            u_trust_g[of + 3] += 2 * (u[of + 3] - last_trust);
            
            if (k != 0) 
            {
                u_trust_g[of - 1] -= 2 * (u[of + 3] - last_trust);
            }
        }

        cost = trust_cost;
        for (int k = 0; k < variable_num_; k++)
        {
            gradient[k] = u_trust_g[k];
        }
    }


    // 计算安全飞行走廊成本
    void MPCC::calcSFCCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient)
    {
        cost = 0.0;
        gradient.resize(variable_num_);

        double sfc_cost = 0.0;
        Eigen::Matrix<double, u_dim_ * n_step_ + n_step_ + 1, 1> sfc_g;
        sfc_g.setZero();

        std::vector<int> state2hPloy_idx;
        calcState2hPloyIdx(u, state2hPloy_idx);

        std::vector<int> ploy_idx;
        ploy_idx.resize(3);
        double dist, dist_2;
        int planes_size;
        Eigen::Vector3d cur_pos, plane_vec, plane_pos;
        for (int k = 0; k < n_step_; k++)
        {
            cur_pos = state_[k].block(0, 0, 3, 1);

            // 选择当前位置以及前后总共三个多面体
            ploy_idx[0] = state2hPloy_idx[k] - 1;
            ploy_idx[1] = state2hPloy_idx[k];
            ploy_idx[2] = state2hPloy_idx[k] + 1;


            // 如果不在多面体内，则直接根据当前索引计算
            if (!isInSFC(ploy_idx, cur_pos))
            {
                planes_size = hPolys_[ploy_idx[1]].cols();
                for (int plane_idx = 0; plane_idx < planes_size; plane_idx++)
                {
                    plane_vec = hPolys_[state2hPloy_idx[k]].block(0, plane_idx, 3, 1);
                    plane_pos = hPolys_[state2hPloy_idx[k]].block(3, plane_idx, 3, 1);

                    plane_vec.normalize();

                    dist = plane_vec.dot(cur_pos - plane_pos) + 0.5;

                    if (dist > 0.0)
                    {
                        dist_2 = dist * dist;
                        
                        sfc_cost += dist_2;

                        sfc_g.block(0, 0, u_dim_ * n_step_, 1) += (2 * dist * plane_vec.transpose() * state_g_[k].block(0, 0, 3, u_dim_ * n_step_)).transpose();
                    }
                }
            }
        }
        
        cost = sfc_cost;
        for (int k = 0; k < variable_num_; k++)
        {
            gradient[k] = sfc_g[k];
        }
    }


    // 计算速度障碍成本
    void MPCC::calcOVCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient)
    {
        cost = 0.0;
        gradient.resize(variable_num_);

        double ov_cost = 0.0;
        Eigen::Matrix<double, u_dim_ * n_step_ + n_step_ + 1, 1> ov_g;
        ov_g.setZero();

        // ob_dynamics_
        double cr, cor_vel_length;
        Eigen::Vector2d cor_vel, cur_pos, cur_vel, dyn_pos, dyn_vel;
        for (int k = 0; k < n_step_; k++)
        {
            cur_pos = state_[k].block(0, 0, 2, 1);
            cur_vel = state_[k].block(3, 0, 2, 1);
            for (auto &ob_dy: ob_dynamics_)
            {
                cr = ob_dy.block(0, 0, 2, 1).norm() + 0.50;
                dyn_vel = ob_dy.block(6, 0, 2, 1);
                dyn_pos = ob_dy.block(3, 0, 2, 1) + k * dyn_vel * dt_;

                // 如果需要调整速度
                if (getCorVel(cor_vel, cur_pos, cur_vel, dyn_pos, dyn_vel, cr))
                {
                    cor_vel_length = cor_vel.norm();

                    ov_cost += cor_vel_length * cor_vel_length;
                    ov_g.block(0, 0, u_dim_ * n_step_, 1) += (2 * cor_vel_length * cor_vel.transpose() * state_g_[k].block(3, 0, 2, u_dim_ * n_step_)).transpose();
                }
            }
        }

        cost = ov_cost;
        for (int k = 0; k < variable_num_; k++)
        {
            gradient[k] = ov_g[k];
        }
    }


    bool MPCC::getCorVel(Eigen::Vector2d &cor_vel,
                         Eigen::Vector2d &cur_pos,
                         Eigen::Vector2d &cur_vel,
                         Eigen::Vector2d &dyn_pos,
                         Eigen::Vector2d &dyn_vel,
                         double &circle_radius)
    {
        // 如果在碰撞域内怎么办

        // 圆的半径 ---- 速度系
        double cr = circle_radius * dt_inv_;

        // 圆心坐标 ---- 速度系
        Eigen::Vector2d coc_pos = (dyn_pos - cur_pos) * dt_inv_;

        if (coc_pos.norm() < cr)
        {
            return false;
        }

        // 圆心与坐标原点连线的角度
        double coc_angle = std::atan2(coc_pos(1), coc_pos(0));

        // 圆心与坐标原点连线的长度
        double coc_length = coc_pos.norm();

        // 与圆切线的顺时针/逆时针增加的角度
        double delta_angle = std::asin((cr / coc_length));

        // 相对速度
        Eigen::Vector2d rv = cur_vel - dyn_vel;

        // 相对速度的大小
        double rv_length = rv.norm();

        // 相对速度的角度
        double rv_angle = std::atan2(rv(1), rv(0));

        // 首先，根据角度判断是否可能发生碰撞
        double diff_angle = std::fmod(rv_angle - coc_angle + M_PI, 2 * M_PI) - M_PI;
        double abs_diff_angle = std::abs(diff_angle);
        if (abs_diff_angle > delta_angle)
        {
            return false;
        }
        
        // 然后，根据是否在圆内判断是否可能发生碰撞，再根据长度是否大于coc判断是否可能发生碰撞
        double diff_length = (rv - coc_pos).norm();
        if (diff_length > cr && rv_length < coc_length)
        {
            return false;
        }

        // 需要调整的角度和调整后的向量
        double cor_delta_angle = delta_angle - abs_diff_angle;;
        double cor_angle;
        Eigen::Vector2d cor_vector;

        // 发生碰撞，计算需要改变速度
        if (diff_angle > 0)         // 逆时针旋转
        {
            cor_angle = rv_angle - cor_delta_angle;

        }
        else                        // 顺时针旋转
        {
            cor_angle = rv_angle + cor_delta_angle;
        }

        if (cor_angle > M_PI)
        {
            cor_angle -= 2 * M_PI;
        }
        else
        {
            cor_angle += 2 * M_PI;
        }

        cor_vector << std::cos(cor_angle), std::sin(cor_angle);
        cor_vector = cor_vector * rv_length;

        cor_vel = cor_vector - rv;

        return true;
    }


    void MPCC::combineCost(const std::vector<double> &u, double &cost, std::vector<double> &gradient)
    {
        cost = 0.0;
        gradient.resize(variable_num_);

        // 计算当前输入对应的状态
        calcState(u);

        // 计算当前状态的偏航角
        calcStateYaw();

        // 计算参考位姿（reference theta），切向向量（tangent vector）以及参考位姿和实际位姿的误差向量（error vector）
        calcRefTagAndError(u);

        // 计算滞后误差
        double lag_cost = 0.0;
        calcLagCost(u, lag_cost, g_lag_);

        // 计算轮廓误差
        double contour_cost = 0.0;
        calcContourCost(u, contour_cost, g_contour_);

        // 计算跟踪进度成本
        double progress_cost = 0.0;
        calcProgressCost(u, progress_cost, g_progress_);

        // 计算与参考曲线的偏航角成本
        double r_yaw_cost = 0.0;
        calcRYawCost(u, r_yaw_cost, g_r_yaw_);

        // 计算速度的偏航角成本
        double v_yaw_cost = 0.0;
        calcVYawCost(u, v_yaw_cost, g_v_yaw_);

        // 计算控制输入中速率变化的成本
        double rate_cost = 0.0;
        calcRateCost(u, rate_cost, g_u_rate_);

        // 计算控制输入中推力变化的成本
        double trust_cost = 0.0;
        calcTrustCost(u, trust_cost, g_u_trust_);

        // 计算安全飞行走廊成本
        double sfc_cost = 0.0;
        calcSFCCost(u, sfc_cost, g_sfc_);

        // 计算速度障碍成本
        double ov_cost = 0.0;
        calcOVCost(u, ov_cost, g_ov_);

        // 计算总成本
        cost = cost_w_(0) * lag_cost
             + cost_w_(1) * contour_cost
             + cost_w_(2) * progress_cost
             + cost_w_(3) * r_yaw_cost
             + cost_w_(4) * v_yaw_cost             
             + cost_w_(5) * rate_cost
             + cost_w_(6) * trust_cost
             + cost_w_(7) * sfc_cost
             + cost_w_(8) * ov_cost;

        // 计算总梯度
        for (int k = 0; k < variable_num_; k++)
        {
            gradient[k] = cost_w_(0) * g_lag_[k]
                        + cost_w_(1) * g_contour_[k]
                        + cost_w_(2) * g_progress_[k]
                        + cost_w_(3) * g_r_yaw_[k]
                        + cost_w_(4) * g_v_yaw_[k]
                        + cost_w_(5) * g_u_rate_[k]
                        + cost_w_(6) * g_u_trust_[k]
                        + cost_w_(7) * g_sfc_[k]
                        + cost_w_(8) * g_ov_[k];
        }
    }


    double MPCC::costFunction(const std::vector<double> &u, std::vector<double> &gradient, MPCC *instance)
    {
        double cost;

        instance->combineCost(u, cost, gradient);

        return cost;
    }


    int MPCC::solver(const Eigen::Matrix<double, x_dim_, 1> &state,
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
                     Eigen::Matrix<double, n_step_ + 1, 1> &t_index)
    {
        // 初始解
        // for (int k = 0; k < n_step_; k++) 
        // {
        //     uv_[k * u_dim_ + 0] = last_u(0);
        //     uv_[k * u_dim_ + 1] = last_u(1);
        //     uv_[k * u_dim_ + 2] = last_u(2);
        //     uv_[k * u_dim_ + 3] = last_u(3);
        // }
        for (int k = 0; k < n_step_; k++) 
        {
            uv_[k * u_dim_ + 0] = predict_u(k, 0);
            uv_[k * u_dim_ + 1] = predict_u(k, 1);
            uv_[k * u_dim_ + 2] = predict_u(k, 2);
            uv_[k * u_dim_ + 3] = predict_u(k, 3);
        }

        for (int k = 0; k < n_step_; k++) 
        {
            uv_[n_step_ * u_dim_ + k] = t_index[k + 1] - t_index[k];
        }
        uv_[n_step_ * u_dim_ + n_step_] = 0.00;
        ub_[n_step_ * u_dim_ + n_step_] = tmax;

        opt_.set_lower_bounds(lb_);
        opt_.set_upper_bounds(ub_);

        // 限制初始控制量的范围
        for (int i = 0; i < (int)uv_.size(); i++)
        {
            if (uv_[i] > ub_[i]) 
            {
                uv_[i] = ub_[i];
            } 
            else if (uv_[i] < lb_[i]) 
            {
                uv_[i] = lb_[i];
            }
        }

        // 相关参数
        init_state_ = state;
        ctrl_pts_ = ctrl_pts;
        v_ctrl_pts_ = v_ctrl_pts;
        ts_ = ts;
        t_max_ = tmax;
        last_u_ = last_u;
        ref_traj_->setUniformBspline(ctrl_pts, 3, ts);

        // 安全飞行走廊
        hPolys_ = hPolys;
        path2hPloy_idx_ = path2hPloy_idx;

        // 动态物体
        ob_dynamics_ = ob_dynamics;
        
        double minf;
        try
        {
            // 解决优化问题
            opt_.optimize(uv_, minf);

            // 从优化变量中获取控制输入
            for (int k = 0; k < n_step_; k++) 
            {
                predict_u(k, 0) = uv_[k * u_dim_ + 0];
                predict_u(k, 1) = uv_[k * u_dim_ + 1];
                predict_u(k, 2) = uv_[k * u_dim_ + 2];
                predict_u(k, 3) = uv_[k * u_dim_ + 3];
            }
            
            // 从优化变量中获取状态
            for (int k = 0; k < n_step_; k++) 
            {
                x_predict.row(k) = state_[k];
            }

            // 从优化变量中获取进度
            t_index[0] = uv_[u_dim_ * n_step_ + n_step_];
            for (int k = 0; k < n_step_; k++) 
            {
                t_index[k + 1] = t_index[k] + uv_[u_dim_ * n_step_ + k];
            }

            return EXIT_SUCCESS;
        } 
        catch(std::exception &e) 
        {
            // 从优化变量中获取控制输入
            for (int k = 0; k < n_step_; k++) 
            {
                predict_u(k, 0) = uv_[k * u_dim_ + 0];
                predict_u(k, 1) = uv_[k * u_dim_ + 1];
                predict_u(k, 2) = uv_[k * u_dim_ + 2];
                predict_u(k, 3) = uv_[k * u_dim_ + 3];
            }

            // 从优化变量中获取状态
            for (int k = 0; k < n_step_; k++) 
            {
                x_predict.row(k) = state_[k];
            }
            
            // 从优化变量中获取进度
            t_index[0] = uv_[u_dim_ * n_step_ + n_step_];
            for (int k = 0; k < n_step_; k++) 
            {
                t_index[k + 1] = t_index[k] + uv_[u_dim_ * n_step_ + k];
            }

            return EXIT_FAILURE;
        }
    } 


    bool MPCC::isInSFC(std::vector<int> ploy_idx,  Eigen::Vector3d &cur_pos)
    {
        bool isInPloys = false;
        for (auto &idx: ploy_idx)
        {
            // 如果索引不合法，直接跳过
            if (idx < 0 || idx >= (int)hPolys_.size())
            {
                continue;
            }

            // 当前位置是否在索引为idx的多面体内
            bool isInPloy = true;
            int planes_size = hPolys_[idx].cols();
            // 遍历多面体的每一个平面，判断当前位置是否在平面外
            for (int plane_idx = 0; plane_idx < planes_size; plane_idx++)
            {
                Eigen::Vector3d plane_vec = hPolys_[idx].block(0, plane_idx, 3, 1);
                Eigen::Vector3d plane_pos = hPolys_[idx].block(3, plane_idx, 3, 1);

                double dist = plane_vec.dot(cur_pos - plane_pos);

                // 只要在一个平面外，则当前位置不在多面体内
                if (dist > 0.0)
                {
                    isInPloy = false;
                    break;
                }
            }

            // 如果在多面体内，直接跳出循环
            if (isInPloy)
            {
                isInPloys = true;
                break;
            }
        }

        return isInPloys;
    }
}
