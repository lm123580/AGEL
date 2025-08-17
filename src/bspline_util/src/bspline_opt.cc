#include "bspline_opt.h"

namespace AGEL
{
    BsplineOptimizer::BsplineOptimizer(ros::NodeHandle &nh) 
    {
        nh.param("global_motion/cost_w_smooth", w_smooth_, 100.0);
        nh.param("global_motion/cost_w_arc",    w_arc_, 100.0);
        nh.param("global_motion/cost_w_feasi",  w_feasi_, 50.0);
        nh.param("global_motion/cost_w_start",  w_start_, 20.0);
        nh.param("global_motion/cost_w_end",    w_end_, 20.0);
        
        nh.param("global_motion/max_vel", max_vel_, 1.00);
        nh.param("global_motion/max_acc", max_acc_, 0.50);
        
        nh.param("global_motion/max_iteration_num",  max_iteration_num_, 500);
        nh.param("global_motion/max_iteration_time", max_iteration_time_, 0.005);

        nh.param("global_motion/bspline_degree", bspline_degree_, 3);
    }


    BsplineOptimizer::~BsplineOptimizer() = default;


    void BsplineOptimizer::setBoundaryStates(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                             const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {
        start_pos_ = start_pos;
        start_vel_ = start_vel;
        start_acc_ = start_acc;
        end_pos_ = end_pos;
        end_vel_ = end_vel;
        end_acc_ = end_acc;
    }


    int BsplineOptimizer::solver(Eigen::MatrixXd &points, double &dt)
    {
        control_points_ = points;
        knot_span_ = dt;

        dim_ = control_points_.cols();
        point_num_ = control_points_.rows();
        variable_num_ = point_num_ * dim_;

        g_smoothness_.resize(variable_num_);
        g_arc_.resize(variable_num_);
        g_feasibility_.resize(variable_num_);
        g_start_.resize(variable_num_);
        g_end_.resize(variable_num_);

        opt_ = nlopt::opt(nlopt::LD_AUGLAG, variable_num_);
        opt_.set_min_objective((nlopt::vfunc)&BsplineOptimizer::costFunction, this);
        opt_.set_maxeval(max_iteration_num_);
        opt_.set_maxtime(max_iteration_time_);
        opt_.set_xtol_rel(1e-16);
        opt_.set_ftol_rel(1e-12);

        std::vector<double> x(variable_num_);
        for (Eigen::Index i = 0; i < point_num_; ++i)
        {
            for (Eigen::Index j = 0; j < dim_; ++j) 
            {
                x[dim_ * i + j] = control_points_(i, j);
            }
        }

        try 
        {
            double final_cost;
            opt_.optimize(x, final_cost);

            for (Eigen::Index i = 0; i < point_num_; ++i)
            {
                for (Eigen::Index j = 0; j < dim_; ++j) 
                {
                    points(i, j) = x[dim_ * i + j];
                }
            }

            return EXIT_SUCCESS;
        } 
        catch (std::exception &e) 
        {
            std::cout << "std exception\t" << e.what() << std::endl;
        
            return EXIT_FAILURE;
        }
    }


    void BsplineOptimizer::combineCost(const std::vector<double> &x, std::vector<double> &grad,
                                       double &cost)
    {
        cost = 0.0;
        grad.resize(variable_num_);
        std::fill(grad.begin(), grad.end(), 0.0);

        double smoothness_cost = 0.0;
        calcSmoothnessCost(x, smoothness_cost, g_smoothness_);

        double arc_cost = 0.0;
        calcArcCost(x, arc_cost, g_arc_);

        double start_cost = 0.0;
        calcStartCost(x, start_cost, g_start_);

        double end_cost = 0.0;
        calcEndCost(x, end_cost, g_end_);

        double feasibility_cost = 0.0;
        calcFeasibilityCost(x, feasibility_cost, g_feasibility_);

        cost = w_smooth_ * smoothness_cost + w_arc_ * arc_cost + w_start_ * start_cost + w_end_ * end_cost + w_feasi_ * feasibility_cost;

        for (int i = 0; i < variable_num_; i++)
        {
            grad[i] = w_smooth_ * g_smoothness_[i] + w_arc_ * g_arc_[i] + w_start_ * g_start_[i] + w_end_ * g_end_[i] + w_feasi_ * g_feasibility_[i];
        }
    }


    // 最小化jerk
    void BsplineOptimizer::calcSmoothnessCost(const std::vector<double> &x, double &cost, 
                                              std::vector<double> &gradient)
    {
        cost = 0.0;
        std::fill(gradient.begin(), gradient.end(), 0.0);
        Eigen::Vector3d jerk, temp_j;

        double dx, dy, dz, j_x, j_y, j_z;
        int n_size = (x.size() / 3) - 3, idx;
        for (int i = 0; i < n_size; i++) 
        {
            dx = (x[(i + 3) * 3 + 0] - 3 * x[(i + 2) * 3 + 0] + 3 * x[(i + 1) * 3 + 0] - x[i * 3 + 0]);
            dy = (x[(i + 3) * 3 + 1] - 3 * x[(i + 2) * 3 + 1] + 3 * x[(i + 1) * 3 + 1] - x[i * 3 + 1]);
            dz = (x[(i + 3) * 3 + 2] - 3 * x[(i + 2) * 3 + 2] + 3 * x[(i + 1) * 3 + 2] - x[i * 3 + 2]);
            cost += (dx * dx + dy * dy + dz * dz);

            j_x = 2 * dx;
            j_y = 2 * dy;
            j_z = 2 * dz;

            idx = i * 3;
            gradient[idx++] += -j_x;         // gradient[(i + 0) * 3 + 0] += -j_x;
            gradient[idx++] += -j_y;         // gradient[(i + 0) * 3 + 1] += -j_y;
            gradient[idx++] += -j_z;         // gradient[(i + 0) * 3 + 2] += -j_z;   

            gradient[idx++] += 3.0 * j_x;    // gradient[(i + 1) * 3 + 0] += 3.0 * j_x;
            gradient[idx++] += 3.0 * j_y;    // gradient[(i + 1) * 3 + 1] += 3.0 * j_y;
            gradient[idx++] += 3.0 * j_z;    // gradient[(i + 1) * 3 + 2] += 3.0 * j_z;

            gradient[idx++] += -3.0 * j_x;   // gradient[(i + 2) * 3 + 0] += -3.0 * j_x;
            gradient[idx++] += -3.0 * j_y;   // gradient[(i + 2) * 3 + 1] += -3.0 * j_y;
            gradient[idx++] += -3.0 * j_z;   // gradient[(i + 2) * 3 + 2] += -3.0 * j_z;

            gradient[idx++] += j_x;          // gradient[(i + 3) * 3 + 0] += j_x;
            gradient[idx++] += j_y;          // gradient[(i + 3) * 3 + 1] += j_y;
            gradient[idx++] += j_z;          // gradient[(i + 3) * 3 + 2] += j_z;
        }
        // std::cout << "smoothness_cost: " << cost << std::endl;
    }


    // 弧长参数化
    void BsplineOptimizer::calcArcCost(const std::vector<double> &x, double &cost, 
                                       std::vector<double> &gradient)
    {
        cost = 0.0;
        std::fill(gradient.begin(), gradient.end(), 0.0);

        double dt = knot_span_;
        Eigen::Vector3d q0, q2, v, dv;

        int n_size = (x.size() / 3);
        for (int i = 2; i < n_size; i++) 
        {
            q0 << x[(i - 2) * 3 + 0], x[(i - 2) * 3 + 1], x[(i - 2) * 3 + 2];
            q2 << x[i * 3 + 0], x[i * 3 + 1], x[i * 3 + 2];
            v = 1.0 / (2 * dt) * (q2 - q0);
            dv = v - v.normalized();

            cost += dv.squaredNorm();

            gradient[(i - 2) * 3 + 0] += (-1.0) * 2 * dv[0] / dt;
            gradient[(i - 2) * 3 + 1] += (-1.0) * 2 * dv[1] / dt;
            gradient[(i - 2) * 3 + 2] += (-1.0) * 2 * dv[2] / dt;

            gradient[i * 3 + 0] += (1.0) * 2 * dv[0] / dt;
            gradient[i * 3 + 1] += (1.0) * 2 * dv[1] / dt;
            gradient[i * 3 + 2] += (1.0) * 2 * dv[2] / dt;
        }
        
        // std::cout << "Arc cost: " << cost << std::endl;
    }

    // 最小化起点误差
    void BsplineOptimizer::calcStartCost(const std::vector<double> &x, double &cost,
                                         std::vector<double> &gradient) 
    {
        cost = 0.0;
        std::fill(gradient.begin(), gradient.end(), 0.0);
        
        double dt = knot_span_;
        Eigen::Vector3d q1, q2, q3, dq;
        q1 << x[0], x[1], x[2];
        q2 << x[3], x[4], x[5];
        q3 << x[6], x[7], x[8];

        // 起点位置
        static const double w_pos = 50.0;
        dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - start_pos_;
        cost += w_pos * dq.squaredNorm();

        gradient[0] += w_pos * 2 * dq[0] * (1 / 6.0);
        gradient[1] += w_pos * 2 * dq[1] * (1 / 6.0);
        gradient[2] += w_pos * 2 * dq[2] * (1 / 6.0);

        gradient[3] += w_pos * 2 * dq[0] * (4 / 6.0);
        gradient[4] += w_pos * 2 * dq[1] * (4 / 6.0);
        gradient[5] += w_pos * 2 * dq[2] * (4 / 6.0);

        gradient[6] += w_pos * 2 * dq[0] * (1 / 6.0);
        gradient[7] += w_pos * 2 * dq[1] * (1 / 6.0);
        gradient[8] += w_pos * 2 * dq[2] * (1 / 6.0);

        // 起点速度
        dq = 1 / (2 * dt) * (q3 - q1) - start_vel_;
        cost += dq.squaredNorm();

        gradient[0] += 2 * dq[0] * (-1.0) / (2 * dt);
        gradient[1] += 2 * dq[1] * (-1.0) / (2 * dt);
        gradient[2] += 2 * dq[2] * (-1.0) / (2 * dt);

        gradient[6] += 2 * dq[0] * 1.0 / (2 * dt);
        gradient[7] += 2 * dq[1] * 1.0 / (2 * dt);
        gradient[8] += 2 * dq[2] * 1.0 / (2 * dt);

        // 起点加速度
        dq = 1 / (dt * dt) * (q1 - 2 * q2 + q3) - start_acc_;
        cost += dq.squaredNorm();

        gradient[0] += 2 * dq[0] * 1.0 / (dt * dt);
        gradient[1] += 2 * dq[1] * 1.0 / (dt * dt);
        gradient[2] += 2 * dq[2] * 1.0 / (dt * dt);

        gradient[3] += 2 * dq[0] * (-2.0) / (dt * dt);
        gradient[4] += 2 * dq[1] * (-2.0) / (dt * dt);
        gradient[5] += 2 * dq[2] * (-2.0) / (dt * dt);

        gradient[6] += 2 * dq[0] * 1.0 / (dt * dt);
        gradient[7] += 2 * dq[1] * 1.0 / (dt * dt);
        gradient[8] += 2 * dq[2] * 1.0 / (dt * dt);
    }


    // 最小化终点误差
    void BsplineOptimizer::calcEndCost(const std::vector<double> &x, double &cost,
                                       std::vector<double> &gradient) 
    {
        cost = 0.0;
        std::fill(gradient.begin(), gradient.end(), 0.0);
        
        double dt = knot_span_, gradient_size = gradient.size();
        std::vector<int> idx;
        for (int i = gradient_size - 9; i <= gradient_size - 1; i++)
        {
            idx.push_back(i);
        }

        idx.push_back(gradient_size);
        Eigen::Vector3d q1, q2, q3, dq;
        q3 << x[idx[0]], x[idx[1]], x[idx[2]];
        q2 << x[idx[3]], x[idx[4]], x[idx[5]];
        q1 << x[idx[6]], x[idx[7]], x[idx[8]];

        // 终点位置
        static const double w_pos = 50.0;
        dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - end_pos_;
        cost += dq.squaredNorm();

        gradient[idx[6]] += 2 * w_pos * dq[0] * (1 / 6.0);
        gradient[idx[7]] += 2 * w_pos * dq[1] * (1 / 6.0);
        gradient[idx[8]] += 2 * w_pos * dq[2] * (1 / 6.0);

        gradient[idx[3]] += 2 * w_pos * dq[0] * (4 / 6.0);
        gradient[idx[4]] += 2 * w_pos * dq[1] * (4 / 6.0);
        gradient[idx[5]] += 2 * w_pos * dq[2] * (4 / 6.0);

        gradient[idx[0]] += 2 * w_pos * dq[0] * (1 / 6.0);
        gradient[idx[1]] += 2 * w_pos * dq[1] * (1 / 6.0);
        gradient[idx[2]] += 2 * w_pos * dq[2] * (1 / 6.0);

        // 终点速度
        dq = 1 / (2 * dt) * (q1 - q3) - end_vel_;
        cost += dq.squaredNorm();

        gradient[idx[6]] += 2 * dq[0] * (-1.0) / (2 * dt);
        gradient[idx[7]] += 2 * dq[1] * (-1.0) / (2 * dt);
        gradient[idx[8]] += 2 * dq[2] * (-1.0) / (2 * dt);

        gradient[idx[0]] += 2 * dq[0] * 1.0 / (2 * dt);
        gradient[idx[1]] += 2 * dq[1] * 1.0 / (2 * dt);
        gradient[idx[2]] += 2 * dq[2] * 1.0 / (2 * dt);

        // 终点加速度
        dq = 1 / (dt * dt) * (q1 - 2 * q2 + q3) - end_acc_;
        cost += dq.squaredNorm();

        gradient[idx[6]] += 2 * dq[0] * 1.0 / (dt * dt);
        gradient[idx[7]] += 2 * dq[1] * 1.0 / (dt * dt);
        gradient[idx[8]] += 2 * dq[2] * 1.0 / (dt * dt);

        gradient[idx[3]] += 2 * dq[0] * (-2.0) / (dt * dt);
        gradient[idx[4]] += 2 * dq[1] * (-2.0) / (dt * dt);
        gradient[idx[5]] += 2 * dq[2] * (-2.0) / (dt * dt);

        gradient[idx[0]] += 2 * dq[0] * 1.0 / (dt * dt);
        gradient[idx[1]] += 2 * dq[1] * 1.0 / (dt * dt);
        gradient[idx[2]] += 2 * dq[2] * 1.0 / (dt * dt);
    }


    // 惩罚项
    void BsplineOptimizer::calcFeasibilityCost(const std::vector<double> &x, double &cost,
                                               std::vector<double> &gradient) 
    {
        cost = 0.0;
        std::fill(gradient.begin(), gradient.end(), 0.0);

        const double dt_inv = 1 / knot_span_;
        const double dt_inv2 = dt_inv * dt_inv;

        int n_size = (x.size() / 3);
        double sign, tmp;

        // 速度惩罚
        double v, vd;
        for (int i = 0; i < n_size - 1; i++) 
        {
            for (int j = 0; j < 3; j++)
            {
                v = (x[(i + 1) * 3 + j] - x[i * 3 + j]) * dt_inv;
                vd = std::fabs(v) - max_vel_;
                if (vd > 0.0) 
                {
                    cost += vd * vd;
                    sign = v > 0 ? 1.0 : -1.0;
                    tmp = 2 * vd * sign * dt_inv;
                    gradient[i * 3 + j] += -tmp;
                    gradient[(i + 1) * 3 + j] += tmp;
                }                
            }
        }

        // 加速度惩罚
        double a, ad;
        for (int i = 0; i < n_size - 2; i++) 
        {
            for (int j = 0; j < 3; j++) 
            {
                a = (x[(i + 2) * 3 + j] - 2 * x[(i + 1) * 3 + j] + x[i * 3 + j]) * dt_inv2;
                ad = fabs(a) - max_acc_;
                if (ad > 0.0) 
                {
                    cost += ad * ad;
                    sign = a > 0 ? 1.0 : -1.0;
                    tmp = 2 * ad * sign * dt_inv2;
                    gradient[i * 3 + j] += tmp;
                    gradient[(i + 1) * 3 + j] += -2 * tmp;
                    gradient[(i + 2) * 3 + j] += tmp;
                }
            }
        }
    }


    double BsplineOptimizer::costFunction(const std::vector<double> &x, std::vector<double> &grad,
                                          BsplineOptimizer *instance)
    {
        double cost;
        
        instance->combineCost(x, grad, cost);

        return cost;
    }

}