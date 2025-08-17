#include "system_fsm.h"
#include "px4_interface.h"
#include "map_manager.h"
#include "sdf_map.h"
#include "motion_manager.h"
#include "explore_manager.h"

namespace xxx_xxx
{
    SystemFSM::SystemFSM() = default;
    SystemFSM::~SystemFSM() = default;

    void SystemFSM::init(ros::NodeHandle& nh)
    {
        nh.param("sdf_map/resolution", map_resolution_, 0.1);
        nh.param("camera_util/max_dist", max_ray_length_, 5.0);

        nh.param("system_fsm/hover_height", hover_height_, 1.2);

        max_ray_length_ = max_ray_length_ * 0.9;

        have_tarigger_ = false;
        have_unlock_ = false;

        last_NBP_ = Eigen::Vector3d(1000000, 1000000, 1000000);
        
        state_ = STATE::UNLOCK;
        motion_mode_ = MOTION_MODE::STATIC;

        waiting_pose_ = Eigen::Vector4d::Zero();
        
        px4_interface_.reset(new Px4Interface(nh));
        map_manager_.reset(new MapManager());
        motion_manager_.reset(new MotionManager());
        explore_manager_.reset(new ExploreManager());
        
        map_manager_->init(nh);
        motion_manager_->init(nh, map_manager_, px4_interface_);
        explore_manager_->init(nh, map_manager_, px4_interface_);

        exec_fsm_timer_ = nh.createTimer(ros::Duration(0.020), &SystemFSM::execFSMCallback, this);
        finish_signal_pub_ = std::make_shared<ros::Publisher>(nh.advertise<std_msgs::Empty>("/explore/finish", 10));
        
        target_sub_ = std::make_shared<ros::Subscriber>(nh.subscribe("/move_base_simple/goal", 5, &SystemFSM::targetCallback, this));
    
        start_time_ = ros::Time::now();

        exploring_ = false;
        rotation_pos_ << 0.0, 0.0, 0.0;
        rotation_yaw_ = 0.0;
    }


    void SystemFSM::execFSMCallback(const ros::TimerEvent& event)
    {
        if (have_unlock_ && px4_interface_->get_mode() != "OFFBOARD")
        {
            px4_interface_->set_px4_mode("OFFBOARD");
        }

        switch (state_)
        {
        // 解锁无人机
        case STATE::UNLOCK:
        {
            if (have_unlock_)
            {
                changeState(WAIT_TARIGGER);
            }
            else
            {
                px4_interface_->set_px4_mode("AUTO.LOITER");
                ros::Duration(0.1).sleep();
                px4_interface_->arm();
                ros::Duration(0.1).sleep();

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
            motion_mode_ = MOTION_MODE::HOVER;

            if (have_tarigger_)
            {
                ROS_INFO("<<<<<===============START!!!==================>>>>>");
                changeState(MOTION);
                motion_mode_ = MOTION_MODE::ROTATION;

                start_flag = true;
                start_time_ = ros::Time::now();
            }
            else
            {
                rotation_pos_ << waiting_pose_(0), waiting_pose_(1), waiting_pose_(2);

                px4_interface_->set_pos(waiting_pose_(0), waiting_pose_(1), waiting_pose_(2), waiting_pose_(3));
            }

            break;            
        }

        // 开始运动规划
        case STATE::MOTION:
        {
            explore_manager_->run();
            
            break;
        }

        case FINISH:
        {
            if (start_flag == true)
            {
                end_time_ = ros::Time::now();

                start_flag = false;
            }
            ROS_INFO_THROTTLE(1.0, "<<<<<===============FINISH!!!==================>>>>>\nTOTAL TIME: %.2f S", (end_time_-start_time_).toSec());
            
            if (have_tarigger_)
            {
                have_tarigger_ = false;
            }

            if (motion_mode_ != MOTION_MODE::HOVER)
            {
                motion_mode_ = MOTION_MODE::HOVER;
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
            current_pos = collisionDetection(current_pos);
            px4_interface_->set_pos(current_pos(0), current_pos(1), current_pos(2), current_yaw);

            break;
        }

        default:
            ROS_ERROR("FSM ERROR!!!");
            break;
        }
    }

    Eigen::Vector3d SystemFSM::collisionDetection(Eigen::Vector3d &target_pos)
    {
        int sample_num = 5;
        std::vector<bool> collision_flag;
        std::vector<Eigen::Vector3d> sample_points, collision_points;

        Eigen::Vector3d tmp_p;
        for (int x = -sample_num; x < sample_num; x++)
        {
            for (int y = -sample_num; y < sample_num; y++)
            {
                tmp_p = Eigen::Vector3d(x * map_resolution_, y * map_resolution_, 0.0) + target_pos;
                sample_points.push_back(tmp_p);
            }
        }

        int sample_size = sample_points.size();
        for (int i = 0; i < sample_size; i++)
        {
            tmp_p = sample_points[i];

            if (map_manager_->map_->getInflateOccupancy(tmp_p))
            {
                collision_flag.push_back(true);
                collision_points.push_back(tmp_p);
            }
            else
            {
                collision_flag.push_back(false);
            }
        }

        if (collision_points.size() == 0)
        {
            return target_pos;
        }

        Eigen::Vector3d safe_pos;
        double max_dist = -std::numeric_limits<double>::max();
        for (int i = 0; i < sample_size; i++)
        {
            if (collision_flag[i])  continue ;

            double tmp_dist = 0.0;
            for (auto &cp: collision_points)
            {
                tmp_dist += (sample_points[i] - cp).squaredNorm();
            }

            if (tmp_dist > max_dist)
            {
                max_dist = tmp_dist;
                safe_pos = sample_points[i];
            }
        }

        return safe_pos;
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