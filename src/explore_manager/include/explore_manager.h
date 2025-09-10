#ifndef EXPLORE_MANAGER_H_
#define EXPLORE_MANAGER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <mutex>
#include <thread>

#include <Eigen/Eigen>

namespace AGEL
{
    class MapManager;

    class Px4Interface;

    class MotionManager;

    class ExploreManager
    {
    private:
        enum MOTION_MODE
        {
            INIT,
            ROTATION,
            HOVER,
            MOVING,
            FINISH,
        };

        enum ROTATION_DIRECTION
        {
            NONE,
            CLOCKWISE,
            COUNTERCLOCKWISE,
        };

        std::shared_ptr<MapManager> map_manager_;
        std::shared_ptr<Px4Interface> px4_interface_;
        std::shared_ptr<MotionManager> motion_manager_;


        std::vector<std::string> state_str_ = {"UNLOCK", "WAIT_TARIGGER", "MOTION", "FINISH"};

        MOTION_MODE motion_mode_{MOTION_MODE::INIT};

        // 正在寻找下一个最佳点
        bool calc_nbp_ing_{false};

        // 开始寻找最佳点
        bool calc_nbp_start_{false};

        bool last_is_rotation_{true};

        // 最大采样距离
        double max_ray_length_, delta_yaw_;

        // 旋转方向
        double rotation_direction_, hover_height_;

        // 最后的安全位置
        Eigen::Vector3d last_safe_position_;

        Eigen::Vector3d target_frontier_point_;
        Eigen::Vector2d target_frontier_normal_;

        std::mutex mode_mutex_;

        // 控制的定时器
        ros::Timer control_tiemr_;

        void change2Mode(MOTION_MODE mode);

        double getRotationDirection(Eigen::Vector3d& current_pos, double& current_yaw);

        bool sampledUnknowArea(Eigen::Vector3d& current_pos, double sample_yaw);

        bool frontierExplored(Eigen::Vector3d& point, Eigen::Vector2d& normal);
        
        Eigen::Vector3d collisionDetection(Eigen::Vector3d &target_pos);

        void calcNextTargetFrontier(Eigen::Vector3d start_pos, double current_yaw);

        void getTargetFrontierInfo(Eigen::Vector3d &point, Eigen::Vector2d &normal);
        
        void controlCallback(const ros::TimerEvent &event);


    public:
        ExploreManager();
        ~ExploreManager();

        void run();

        void init(ros::NodeHandle &nh, std::shared_ptr<MapManager> &map_manager, std::shared_ptr<Px4Interface> &px4_interface);

        int getFrontiersNumber();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif