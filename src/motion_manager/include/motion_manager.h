#ifndef MOTION_MANAGER_H_
#define MOTION_MANAGER_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <string>
#include <math.h>

#include <mutex>

#include <Eigen/Eigen>

namespace AGEL
{
    class MapManager;
    class SFC;
    
    // class Astar;
    class Astar;
    class UniformBspline;
    class BsplineOptimizer;

    class Px4Interface;
    class QuadrotorDynamic;
    class MPCC;

    // class Frontier;

    class MotionManager
    {
    private:
        std::shared_ptr<MapManager> map_manager_;
        std::shared_ptr<SFC> sfc_;

        std::mutex bspline_mutex_;
        std::shared_ptr<Astar> astar_;
        std::shared_ptr<UniformBspline> bspline_;
        std::shared_ptr<UniformBspline> bspline_exec_;
        std::shared_ptr<BsplineOptimizer> bspline_opt_;
        
        std::shared_ptr<Px4Interface> px4_interface_;
        std::shared_ptr<QuadrotorDynamic> quadrotor_dynamic_;
        std::shared_ptr<MPCC> mpcc_;

        // std::shared_ptr<Frontier> frontier_;

        // 安全走廊
        std::vector<int> path2hPloy_idx_;
        std::vector<Eigen::MatrixXd> corridor_polys_;

        // MPCC
        bool state_ok_;
        double arc_length_{}, arc_ts_{}, haver_ratio_;
        Eigen::Matrix<double, 9, 1> cost_w_;
        Eigen::Matrix<double, 4, 1> last_u_;
        Eigen::Matrix<double, 15, 4> u_predict_;
        Eigen::Matrix<double, 15, 10> x_predict_;
        Eigen::Matrix<double, 15 + 1, 1> t_index_;

        // VIS
        std::vector<Eigen::Vector3d> predict_state_pos_;
        std::vector<Eigen::Vector4d> predict_state_Q_;
        std::vector<Eigen::Vector3d> global_waypoints_, time_bspline_waypoints_, arc_bspline_waypoints_;
        
        nav_msgs::Path passed_path_;
        visualization_msgs::MarkerArray predict_state_;
        visualization_msgs::MarkerArray astar_traj_, time_bspline_traj_, arc_bspline_traj_;

        // ROS
        ros::Timer path_timer_, vis_timer_;
        std::shared_ptr<ros::Publisher> passed_path_pub_, predict_state_pub_;
        std::shared_ptr<ros::Publisher> astar_pub_, time_bspline_pub_, arc_bspline_pub_;
        
        // 分割长路径
        void splitPath(const std::vector<Eigen::Vector3d> &path, std::vector<Eigen::Vector3d> &split_path);
        
        // Marker设置为Delete
        void deleteMarkerArray(visualization_msgs::MarkerArray &mk_array);

        // 预测状态
        void pubPredictState(const std::vector<Eigen::Vector3d> &pos, const std::vector<Eigen::Vector4d> &Q);

        // A*路径
        void pubAstarTraj(const std::vector<Eigen::Vector3d> &waypoints);
        
        // 时间参数化B样条
        void pubReferTimeBspline(const std::vector<Eigen::Vector3d> &waypoints);

        // 弧长参数化B样条
        void pubReferArcBspline(const std::vector<Eigen::Vector3d> &waypoints);

        // 无人机路径回调
        void pathTimerCallback(const ros::TimerEvent& event);

        // 运动轨迹可视化回调
        void visTimerCallback(const ros::TimerEvent& event);

        struct TimeInfo {
            int num{0};
            double total_time{0};
        };
        int print_num_{0}, print_num2_{0};
        TimeInfo search_path_info_, global_opt_info_, sfc_info_, local_info_;

    public:
        MotionManager();
        ~MotionManager();

        void init(ros::NodeHandle& nh, std::shared_ptr<MapManager> &map_manager, std::shared_ptr<Px4Interface> &px4_interface);

        void resetMotion();

        // 清理残留轨迹
        void visCleaner();

        bool globalPlanning(Eigen::Vector3d &goal, const bool is_replan);
        
        void localPlanning();

        double getGlobalStartYaw();
    
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}


#endif