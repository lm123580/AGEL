#include "system_fsm.h"
#include "px4_interface.h"
#include "map_manager.h"
#include "sdf_map.h"
#include "motion_manager.h"
#include "explore_manager.h"
// 修改两个地方     一个是迁移到EXPLOR    另一个是分离线程的操作
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
        // control_tiemr_ = nh.createTimer(ros::Duration(0.020), &SystemFSM::controlCallback,this);
        // info_timer_ = nh.createTimer(ros::Duration(0.5), &SystemFSM::infomationCallback, this);
        finish_signal_pub_ = std::make_shared<ros::Publisher>(nh.advertise<std_msgs::Empty>("/explore/finish", 10));
        
        target_sub_ = std::make_shared<ros::Subscriber>(nh.subscribe("/move_base_simple/goal", 5, &SystemFSM::targetCallback, this));
    
        start_time_ = ros::Time::now();

        exploring_ = false;
        rotation_pos_ << 0.0, 0.0, 0.0;
        rotation_yaw_ = 0.0;
    }


    void SystemFSM::execFSMCallback(const ros::TimerEvent& event)
    {
        // ros::Time time = ros::Time::now();

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
                
                // Eigen::Vector3d current_pos = px4_interface_->get_pos();
                // rotation_pos_ << waiting_pose_(0), waiting_pose_(1), waiting_pose_(2);
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
            // Eigen::Vector3d c_p = px4_interface_->get_pos();
            // if (!map_manager_->map_->getInflateOccupancy(c_p) && motion_mode_ != MOTION_MODE::MOVING && explore_manager_->getFrontiersNumber() == 0)
            // {
            //     changeState(FINISH);

            //     finish_signal_pub_->publish(std_msgs::Empty());

            //     return ;
            // }

            // // if (motion_mode_ == MOTION_MODE::ROTATION)      std::cout << "STATE IS:    ROTATION" << std::endl;
            // // else if (motion_mode_ == MOTION_MODE::MOVING)   std::cout << "STATE IS:    MOVING" << std::endl;

            // exploring_ = true;

            // double current_yaw;
            // Eigen::Vector3d current_pos, next_point;

            // current_yaw = px4_interface_->get_yaw();
            // current_pos = px4_interface_->get_pos();

            // // 开始运动模式为自旋的情况
            // bool rotation2moving = false;
            // if (motion_mode_ == MOTION_MODE::ROTATION)
            // {
            //     double next_yaw;
            //     bool is_rotation_mode;
                
            //     ros::Time t1 = ros::Time::now();
            //     is_rotation_mode = sampleNextYaw(current_pos, current_yaw, next_yaw);
            //     ros::Time t2 = ros::Time::now();
            //     local_sampling_info_.num++;
            //     local_sampling_info_.total_time += ((t2 - t1).toSec() * 1000.0);
                
            //     if (is_rotation_mode)
            //     {
            //         rotation_yaw_ = next_yaw;

            //         last_NBP_ = Eigen::Vector3d(1000000, 1000000, 1000000);         //  确保无目标
            //         // px4_interface_->set_pos(rotation_pos_(0), rotation_pos_(1), hover_height_, next_yaw);
            //     }
            //     else
            //     {
            //         rotation2moving = true;
            //     }
            // }

            // // 开始运动模式为运动的情况
            // bool replan = false;
            // // bool no_path = false;
            // bool moving2rotation = false;
            // if (motion_mode_ == MOTION_MODE::MOVING || rotation2moving)
            // {
            //     if (last_NBP_[0] > 1000)        // 上一次未找到目标点 
            //     {
            //         explore_manager_->calcNextBestPoint();
            //         explore_manager_->getNextBestPointInfo(last_NBP_, last_NBP_normal_);
            //     }
            //     else                            // 上一次找到目标点，判断是否需要更新目标点
            //     {
            //         // 如果需要改变目标点
            //         if (isChangeNextPoint(last_NBP_, last_NBP_normal_))
            //         {
            //             moving2rotation = true;
            //             explore_manager_->calcNextBestPoint();
            //             explore_manager_->getNextBestPointInfo(last_NBP_, last_NBP_normal_);
            //         }
            //         else
            //         {
            //             replan = true;
            //         }
            //     }
                
            //     if (last_NBP_[0] < 1000)
            //     {
            //         if (motion_manager_->globalPlanning(last_NBP_, replan) == false)
            //         {
            //             explore_manager_->calcNextBestPoint();
            //             explore_manager_->getNextBestPointInfo(last_NBP_, last_NBP_normal_);
            //         }
            //     }
                    
            //     if (motion_mode_ != MOTION_MODE::MOVING)
            //     {
            //         motion_mode_ = MOTION_MODE::MOVING;
            //     }
            // }

            // // // 如果速度足够小，考虑自旋
            // // double vel;
            // // vel = (px4_interface_->get_vel()).norm();
            // if (moving2rotation)
            // {
            //     // true代表自旋，false代表移动
            //     bool is_rotation_mode;
            //     double next_yaw;
            //     is_rotation_mode = sampleNextYaw(current_pos, current_yaw, next_yaw);

            //     if (is_rotation_mode)
            //     {
            //         motion_mode_ = MOTION_MODE::ROTATION;
            //         rotation_yaw_ = next_yaw;

            //         // rotation_pos_ << current_pos(0), current_pos(1), 1.20;
            //         // px4_interface_->set_pos(rotation_pos_(0), rotation_pos_(1), hover_height_, next_yaw);
            //     }
            //     else
            //     {
            //         motion_mode_ = MOTION_MODE::ROTATION;
            //         rotation_yaw_ = current_yaw;
            //         // rotation_pos_ << current_pos(0), current_pos(1), 1.20;
            //     }
            // }

            // exploring_ = false;
            
            // print_num_++;
            // if (print_num_ % 100 == 0)
            // {
            //     std::cout << "局部采样时间：" << (local_sampling_info_.total_time / local_sampling_info_.num) << std::endl;
            // }
            
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


    bool SystemFSM::isChangeNextPoint(Eigen::Vector3d& point, Eigen::Vector2d& normal)
    {
        if (point[0] > 1000)
        {
            return true;
        }

        Eigen::Vector3d current_pos;
        current_pos = px4_interface_->get_pos();

        if ((current_pos - point).norm() < 1.0)
        {
            return true;
        }

        auto &map = map_manager_->map_;
        int state_unknow   = map->UNKNOWN;
        int state_occupied = map->OCCUPIED;

        double update_length = max_ray_length_;
        double delta_length = map_resolution_ * 0.5;

        bool is_change = true;
        Eigen::Vector2i tmp_pos_idx;
        Eigen::Vector2d tmp_pos, tmp_point;

        tmp_point = point.head<2>();
        for (double length = delta_length; length < update_length; length += delta_length)
        {
            tmp_pos = tmp_point + normal * length;
            map->posToIndex(tmp_pos, tmp_pos_idx);

            if (map->get2DState(tmp_pos_idx) == state_occupied)
            {
                break;
            }

            if (map->get2DState(tmp_pos_idx) == state_unknow)
            {
                is_change = false;
        
                break;
            }
        }

        return is_change;
    }


    bool SystemFSM::sampleNextYaw(Eigen::Vector3d& current_pos, double& current_yaw, double& next_yaw)
    {
        auto &map = map_manager_->map_;
        int sample_state, state_occupied, state_unknow;

        state_occupied = map->OCCUPIED;
        state_unknow   = map->UNKNOWN;

        double delta_yaw, tmp_yaw, sample_yaw, delta_length;
        Eigen::Vector2d sample_pos;
        Eigen::Vector2i sample_pos_idx;

        delta_yaw = M_PI / 16.0;
        tmp_yaw = 0.0;
        delta_length = map_resolution_ * 0.5;
        while (tmp_yaw < M_PI)
        {
            // 采样右边的点
            sample_yaw = current_yaw + tmp_yaw;
            if (sample_yaw > M_PI) sample_yaw = sample_yaw - (2 * M_PI);
            
            // 对射线上的点采样
            for (double length = delta_length; length < max_ray_length_; length += delta_length)
            {
                sample_pos[0] = current_pos[0] + length * std::cos(sample_yaw);
                sample_pos[1] = current_pos[1] + length * std::sin(sample_yaw);
                map->posToIndex(sample_pos, sample_pos_idx);

                sample_state = map->get2DState(sample_pos_idx);
                if (sample_state == state_occupied)
                {
                    break;
                }
                else if (sample_state == state_unknow)
                {
                    next_yaw = sample_yaw;

                    return true;
                }
            }

            // 采样左边的点
            sample_yaw = current_yaw - tmp_yaw;
            if (sample_yaw < -M_PI) sample_yaw = sample_yaw + (2 * M_PI);

            // 对射线上的点采样
            for (double length = delta_length; length < max_ray_length_; length += delta_length)
            {
                sample_pos[0] = current_pos[0] + length * std::cos(sample_yaw);
                sample_pos[1] = current_pos[1] + length * std::sin(sample_yaw);
                map->posToIndex(sample_pos, sample_pos_idx);

                sample_state = map->get2DState(sample_pos_idx);
                if (sample_state == state_occupied)
                {
                    break;
                }
                else if (sample_state == state_unknow)
                {
                    next_yaw = sample_yaw;

                    return true;
                }
            }

            // 增量
            tmp_yaw += delta_yaw;
        }

        return false ;
    }


    void SystemFSM::controlCallback(const ros::TimerEvent& event)
    {
        if (!have_tarigger_ || motion_mode_ != MOTION_MODE::MOVING)
        {
            motion_manager_->visCleaner();
        }

        // if (exploring_)
        // {
        //     return ;
        // }
        
        if (motion_mode_ == MOTION_MODE::ROTATION)
        {
            Eigen::Vector3d current_pos = px4_interface_->get_pos();
            current_pos[2] = hover_height_;
            
            current_pos = collisionDetection(current_pos);

            px4_interface_->set_pos(current_pos(0), current_pos(1), current_pos(2), rotation_yaw_);

            last_is_rotation_ = true;
        }
        else if (motion_mode_ == MOTION_MODE::MOVING)
        {
            // ros::Time t1 = ros::Time::now();

            if (last_is_rotation_)
            {
                double current_yaw = px4_interface_->get_yaw();
                Eigen::Vector3d current_pos = px4_interface_->get_pos();
                current_pos[2] = hover_height_;
                current_pos = collisionDetection(current_pos);

                double start_yaw = motion_manager_->getGlobalStartYaw();

                if (start_yaw > 100.0)
                {
                    px4_interface_->set_pos(current_pos[0], current_pos[1], hover_height_, current_yaw);

                    return ;
                }

                double diff_yaw;
                if (current_yaw - start_yaw > M_PI)         diff_yaw = 2 * M_PI - (current_yaw - start_yaw);
                else if (start_yaw - current_yaw > M_PI)    diff_yaw = 2 * M_PI - (start_yaw - current_yaw);
                else if (current_yaw > start_yaw)           diff_yaw = current_yaw - start_yaw; 
                else                                        diff_yaw = start_yaw - current_yaw;

                if (diff_yaw > M_PI_4)
                {
                    px4_interface_->set_pos(current_pos[0], current_pos[1], current_pos[2], start_yaw);
                }
                else
                {
                    motion_manager_->resetMotion();
                    last_is_rotation_ = false;
                }
            }
            else
            {
                motion_manager_->localPlanning();
            }
            
            // ros::Time t2 = ros::Time::now();
            
            // std::cout << "Local planning time: "<< (t2-t1).toSec() << std::endl;            
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

        // std::cout << "SAFE POS: " << safe_pos.transpose() << std::endl;
        return safe_pos;
    }


    void SystemFSM::changeState(STATE state)
    {
        ROS_INFO("STATE \033[32m %s \033[0m ===>>> \033[32m %s \033[0m", state_str_[state_].c_str(), state_str_[state].c_str());

        state_ = state;
    }

    void SystemFSM::targetCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        // Eigen::Vector3d current_pos;

        // current_pos = px4_interface_->get_pos();
        // map_manager_->initMapState(current_pos);
        
        have_tarigger_ = true;
    }
}