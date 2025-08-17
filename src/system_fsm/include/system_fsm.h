#ifndef SYSTEM_FSM_H_
#define SYSTEM_FSM_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>

#include <Eigen/Eigen>

namespace AGEL
{
    class Px4Interface;

    class MapManager;
    class MotionManager;
    class ExploreManager;

    class SystemFSM
    {
    private: 
        // 状态 枚举
        enum STATE
        {
            UNLOCK,         // 未解锁
            WAIT_TARIGGER,  // 等待 探索开始 信号
            MOTION,         // 开始探索
            FINISH          // 探索结束
        };

        std::vector<std::string> state_str_ = {"UNLOCK", "WAIT_TARIGGER", "MOTION", "FINISH"};

        std::shared_ptr<Px4Interface> px4_interface_;
        std::shared_ptr<MapManager> map_manager_;
        std::shared_ptr<MotionManager> motion_manager_;
        std::shared_ptr<ExploreManager> explore_manager_;

        STATE state_;

        double hover_height_;
        bool have_unlock_, have_tarigger_, start_flag_;
        Eigen::Vector4d waiting_pose_;

        ros::Time start_time_, end_time_;

        ros::Timer exec_fsm_timer_;
        std::shared_ptr<ros::Subscriber> target_sub_;
        std::shared_ptr<ros::Publisher>  finish_signal_pub_;

        void changeState(STATE state);
        void execFSMCallback(const ros::TimerEvent& event);
        void controlCallback(const ros::TimerEvent& event);
        void targetCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    public:
        SystemFSM();
        
        ~SystemFSM();

        void init(ros::NodeHandle &nh);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif 
