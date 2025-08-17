#ifndef SYSTEM_FSM_H_
#define SYSTEM_FSM_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>

#include <Eigen/Eigen>

namespace xxx_xxx
{
    class Px4Interface;

    class MapManager;
    class MotionManager;
    class ExploreManager;

    class SystemFSM
    {
    private: 
        // 状态 解锁->悬浮/等待开始->   自转模式      ->        目标点模式     ->完成
        enum STATE
        {
            UNLOCK,
            WAIT_TARIGGER,
            MOTION,
            FINISH
        };

        enum MOTION_MODE
        {
            STATIC,
            HOVER,
            ROTATION,
            MOVING
        };

        std::vector<std::string> state_str_ = {"UNLOCK", "WAIT_TARIGGER", "MOTION", "FINISH"};


        std::shared_ptr<Px4Interface> px4_interface_;

        std::shared_ptr<MapManager> map_manager_;
        std::shared_ptr<MotionManager> motion_manager_;
        std::shared_ptr<ExploreManager> explore_manager_;

        STATE state_;
        MOTION_MODE motion_mode_;

        double hover_height_;
        double map_resolution_, max_ray_length_;

        bool have_tarigger_, have_unlock_;
        Eigen::Vector4d waiting_pose_;
        Eigen::Vector2d last_NBP_normal_;
        Eigen::Vector3d start_, target_, last_NBP_;
        
        bool exploring_;
        bool last_is_rotation_;
        double rotation_yaw_;
        Eigen::Vector3d rotation_pos_;

        ros::Timer exec_fsm_timer_, control_tiemr_, info_timer_;
        std::shared_ptr<ros::Subscriber> target_sub_;

        bool start_flag;
        ros::Time start_time_, end_time_;
        std::shared_ptr<ros::Publisher>  finish_signal_pub_;

        void changeState(STATE state);
        void execFSMCallback(const ros::TimerEvent& event);
        void controlCallback(const ros::TimerEvent& event);
        double getCurrentYaw();
        bool sampleNextYaw(Eigen::Vector3d& current_pos, double& current_yaw, double& next_yaw);
        bool isChangeNextPoint(Eigen::Vector3d& point, Eigen::Vector2d& normal);
        void targetCollisionCheck();
        void infomationCallback(const ros::TimerEvent& event);
        void targetCallback(const geometry_msgs::PoseStampedConstPtr& msg);

        Eigen::Vector3d collisionDetection(Eigen::Vector3d &safe_pos);

        struct TimeInfo {
            int num{0};
            double total_time{0};
        };

        int print_num_{0};
        TimeInfo frontier_sampling_info_, local_sampling_info_;
    
    public:
        SystemFSM();
        
        ~SystemFSM();

        void init(ros::NodeHandle &nh);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif 
