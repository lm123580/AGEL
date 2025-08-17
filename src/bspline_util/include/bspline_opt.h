#ifndef _BSPLINE_OPT_H_
#define _BSPLINE_OPT_H_

#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <nlopt.hpp>
#include <Eigen/Eigen>

namespace AGEL
{
    class BsplineOptimizer
    {
    private:
        // 待优化B样条参数
        double knot_span_{};
        int bspline_degree_{};
        Eigen::MatrixXd control_points_;

        // 终端状态约束
        Eigen::Vector3d start_pos_, end_pos_;
        Eigen::Vector3d start_vel_, end_vel_;
        Eigen::Vector3d start_acc_, end_acc_;
        
        // 优化参数
        double w_smooth_{}, w_arc_{}, w_feasi_{}, w_start_{}, w_end_{};
        double max_vel_{}, max_acc_{};

        int max_iteration_num_{};
        double max_iteration_time_{};


        // Data of opt
        std::vector<double> g_smoothness_, g_arc_, g_feasibility_, g_start_, g_end_;

        Eigen::Index variable_num_{};                    // optimization variables
        Eigen::Index dim_{};
        Eigen::Index point_num_{};
        int iter_num_{};                        // iteration of the solver
        double min_cost_{};

        nlopt::opt opt_;

        void combineCost(const std::vector<double> &x, std::vector<double> &grad,
                         double &cost);

        void calcSmoothnessCost(const std::vector<double> &x, double &cost, 
                                std::vector<double> &gradient);

        void calcArcCost(const std::vector<double> &x, double &cost, 
                         std::vector<double> &gradient);

        void calcStartCost(const std::vector<double> &x, double &cost,
                           std::vector<double> &gradient);
        
        void calcEndCost(const std::vector<double> &x, double &cost,
                         std::vector<double> &gradient);
        
        void calcFeasibilityCost(const std::vector<double> &x, double &cost,
                                 std::vector<double> &gradient);

        static double costFunction(const std::vector<double> &x, std::vector<double> &grad,
                                   BsplineOptimizer *instance);

    public:
        BsplineOptimizer(ros::NodeHandle &nh);
        ~BsplineOptimizer();

        void setBoundaryStates(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                               const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

        int solver(Eigen::MatrixXd &points, double &dt);
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif
