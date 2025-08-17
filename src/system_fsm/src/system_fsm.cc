#include "system_fsm.h"
#include "px4_interface.h"
#include "map_manager.h"
#include "sdf_map.h"
#include "motion_manager.h"
#include "explore_manager.h"

namespace AGEL
{
    SystemFSM::SystemFSM() = default;
    SystemFSM::~SystemFSM() = default;

    void SystemFSM::init(ros::NodeHandle& nh)
    {
        // 初始化参数
        nh.param("system_fsm/hover_height", hover_height_, 1.2);

        have_tarigger_ = false;
        have_unlock_ = false;
        
        state_ = STATE::UNLOCK;
        waiting_pose_ = Eigen::Vector4d::Zero();

        px4_interface_.reset(new Px4Interface(nh));     // px4 控制接口
        map_manager_.reset(new MapManager());           // 地图管理
        motion_manager_.reset(new MotionManager());     // 运动管理
        explore_manager_.reset(new ExploreManager());   // 探索管理

        map_manager_->init(nh);
        motion_manager_->init(nh, map_manager_, px4_interface_);
        explore_manager_->init(nh, map_manager_, px4_interface_);

        exec_fsm_timer_ = nh.createTimer(ros::Duration(0.020), &SystemFSM::execFSMCallback, this);

        finish_signal_pub_ = std::make_shared<ros::Publisher>(nh.advertise<std_msgs::Empty>("/explore/finish", 10));
        target_sub_ = std::make_shared<ros::Subscriber>(nh.subscribe("/move_base_simple/goal", 5, &SystemFSM::targetCallback, this));
    
        start_time_ = ros::Time::now();
    }


    void SystemFSM::execFSMCallback(const ros::TimerEvent& event)
    {
        if (have_unlock_ && px4_interface_->get_mode() != "OFFBOARD")
        {
            px4_interface_->set_px4_mode("OFFBOARD");
        }

        switch (state_)
        {
        // 未解锁状态
        case STATE::UNLOCK:
        {
            if (have_unlock_)
            {
                changeState(WAIT_TARIGGER);
            }
            else
            {
                // 解锁无人机
                px4_interface_->set_px4_mode("AUTO.LOITER");
                ros::Duration(0.1).sleep();
                px4_interface_->arm();
                ros::Duration(0.1).sleep();

                // 设置悬停位置
                double current_yaw;
                Eigen::Vector3d current_pos;
                current_yaw = px4_interface_->get_yaw();
                current_pos = px4_interface_->get_pos();
                waiting_pose_ << current_pos(0), current_pos(1), hover_height_, current_yaw; 

                have_unlock_ = true;
            }
            
            break;            
        }
        
        // 悬浮并等待
        case STATE::WAIT_TARIGGER:
        {
            if (have_tarigger_) // 如果有触发，则进入运动模式 开始探索
            {
                ROS_INFO("<<<<<===============START!!!==================>>>>>");
                changeState(MOTION);

                start_flag_ = true;
                start_time_ = ros::Time::now();
            }
            else                // 否则，保持悬停
            {
                px4_interface_->set_pos(waiting_pose_(0), waiting_pose_(1), waiting_pose_(2), waiting_pose_(3));
            }

            break;            
        }

        case STATE::MOTION:
        {
            // 开始探索
            explore_manager_->run();
            
            break;
        }

        case FINISH:
        {
            if (start_flag_ == true)
            {
                end_time_ = ros::Time::now();

                start_flag_ = false;
            }
            ROS_INFO_THROTTLE(1.0, "<<<<<===============FINISH!!!==================>>>>>\nTOTAL TIME: %.2f S", (end_time_-start_time_).toSec());
            
            if (have_tarigger_)
            {
                have_tarigger_ = false;
            }

            finish_signal_pub_->publish(std_msgs::Empty());

            double current_yaw;
            Eigen::Vector3d current_pos, current_vel;
            current_yaw  = px4_interface_->get_yaw();
            current_pos = px4_interface_->get_pos();
            current_vel = px4_interface_->get_vel();
            if (current_vel.norm() > 0.5)
            {
                current_pos = current_pos + current_vel * 0.05 * 0.5;
            }
            current_pos[2] = 1.2;
            px4_interface_->set_pos(current_pos(0), current_pos(1), current_pos(2), current_yaw);

            break;
        }

        default:
            ROS_ERROR("FSM ERROR!!!");
            break;
        }
    }


    void SystemFSM::changeState(STATE state)
    {
        ROS_INFO("STATE \033[32m %s \033[0m ===>>> \033[32m %s \033[0m", state_str_[state_].c_str(), state_str_[state].c_str());

        state_ = state;
    }

    void SystemFSM::targetCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        have_tarigger_ = true;
    }
}