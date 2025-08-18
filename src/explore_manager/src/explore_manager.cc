#include "sdf_map.h"
#include "map_manager.h"
#include "frontier.h"
#include "px4_interface.h"
#include "motion_manager.h"
#include "explore_manager.h"

namespace AGEL
{
    ExploreManager::ExploreManager() = default;

    ExploreManager::~ExploreManager() = default;

    void ExploreManager::init(ros::NodeHandle& nh, std::shared_ptr<MapManager> &map_manager, std::shared_ptr<Px4Interface> &px4_interface)
    {
        nh.param("system_fsm/hover_height", hover_height_, 1.2);
        nh.param("camera_util/max_dist", max_ray_length_, 4.5);
        nh.param("explore_manager/delta_yaw", delta_yaw_, 0.19635);

        map_manager_ = map_manager;
        px4_interface_ = px4_interface;

        motion_manager_.reset(new MotionManager());

        motion_manager_->init(nh, map_manager_, px4_interface_);

        control_tiemr_ = nh.createTimer(ros::Duration(0.030), &ExploreManager::controlCallback,this);
    }


    void ExploreManager::run()
    {
        // 旋转采样 
        double current_yaw;
        Eigen::Vector3d current_pos;

        current_pos = px4_interface_->get_pos();
        current_pos[2] = hover_height_;
        current_yaw = px4_interface_->get_yaw();

        last_safe_position_ = collisionDetection(current_pos);

        switch (motion_mode_)
        {
        case MOTION_MODE::INIT:
        {
            // 进入旋转状态
            change2Mode(MOTION_MODE::ROTATION);

            break;
        }

        case MOTION_MODE::ROTATION:
        {
            // 旋转采样 
            double next_yaw;

            next_yaw = getRotationDirection(last_safe_position_, current_yaw);

            if (std::isinf(next_yaw))
            {
                calc_nbp_start_ = false;
                calc_nbp_ing_ = false;
                hovering_yaw_ = current_yaw;
                change2Mode(MOTION_MODE::HOVER);
            }
            else
            {
                // 旋转到下一个采样点
                px4_interface_->set_pos(last_safe_position_[0], last_safe_position_[1], last_safe_position_[2], next_yaw);
            }

            break;
        }

        case MOTION_MODE::HOVER:
        {
            // 开始寻找最佳目标点
            if (calc_nbp_start_)
            {
                // 已经寻找完毕
                if (!calc_nbp_ing_)
                {
                    if (getFrontiersNumber() == 0)
                    {
                        change2Mode(MOTION_MODE::FINISH);
                    }

                    getTargetFrontierInfo(target_frontier_point_, target_frontier_normal_);

                    // 如果目标前沿在地图内
                    if (map_manager_->map_->isInMap(target_frontier_point_))
                    {
                        bool replan = false;
                        if (motion_manager_->globalPlanning(target_frontier_point_, replan))
                        {
                            // 规划成功
                            last_is_rotation_ = true;
                            change2Mode(MOTION_MODE::MOVING);
                        }
                    }
                    else
                    {
                        // 目标前沿点不在地图内，重新寻找下一个目标前沿点
                        calc_nbp_start_ = false;
                        calc_nbp_ing_ = false;
                    }
                }
            }
            else
            {
                calc_nbp_start_ = true;
                calc_nbp_ing_ = true;
                std::thread t_calc(&ExploreManager::calcNextTargetFrontier, this, last_safe_position_);
                t_calc.detach();
            }

            px4_interface_->set_pos(last_safe_position_[0], last_safe_position_[1], last_safe_position_[2], current_yaw);

            break;
        }

        case MOTION_MODE::MOVING:
        {
            if (frontierExplored(target_frontier_point_, target_frontier_normal_))
            {
                change2Mode(MOTION_MODE::ROTATION);
            }
            else
            {
                bool replan = true;
                mode_mutex_.lock();
                if (!motion_manager_->globalPlanning(target_frontier_point_, replan))
                {
                    hovering_pose_ = px4_interface_->get_pos();
                    hovering_yaw_ = px4_interface_->get_yaw();
                    change2Mode(MOTION_MODE::ROTATION);
                }
                mode_mutex_.unlock();
            }

            break;
        }

        case MOTION_MODE::FINISH:
        {
            // 结束探索
            ROS_INFO_THROTTLE(1.0, "<<<<<===============FINISH!!!==================>>>>>");

            break;
        }
        
        default:
            break;
        }
    }


    double ExploreManager::getRotationDirection(Eigen::Vector3d& current_pos, double& current_yaw)
    {
        auto &map = map_manager_->map_;

        int sample_state, state_occupied, state_unknow;
        state_occupied = map->OCCUPIED;
        state_unknow   = map->UNKNOWN;

        double map_resolution = map->getResolution();

        double delta_yaw, tmp_yaw, sample_yaw, delta_length;

        delta_yaw = delta_yaw_;
        delta_length = map_resolution * 0.5;

        tmp_yaw = 0.0;

        Eigen::Vector2d sample_pos;
        Eigen::Vector2i sample_pos_idx;

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
                    return sample_yaw + delta_yaw;
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
                    return sample_yaw + delta_yaw;
                }
            }

            // 增量
            tmp_yaw += delta_yaw;
        }

        return std::numeric_limits<double>::infinity();
    }


    bool ExploreManager::frontierExplored(Eigen::Vector3d& point, Eigen::Vector2d& normal)
    {
        if (std::isinf(point[0]))
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
        double resolution = map->getResolution();

        double update_length = max_ray_length_;
        double delta_length = resolution * 0.5;

        bool explored = true;
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
                explored = false;
        
                break;
            }
        }

        return explored;
    }


    void ExploreManager::controlCallback(const ros::TimerEvent &event)
    {
        double current_yaw = px4_interface_->get_yaw();

        if (motion_mode_ == MOTION_MODE::MOVING)
        {
            if (last_is_rotation_)
            {
                double start_yaw = motion_manager_->getGlobalStartYaw();

                if (isinf(start_yaw))
                {
                    px4_interface_->set_pos(last_safe_position_[0], last_safe_position_[1], last_safe_position_[2], current_yaw);

                    return ;
                }

                double diff_yaw;
                if (current_yaw - start_yaw > M_PI)         diff_yaw = 2 * M_PI - (current_yaw - start_yaw);
                else if (start_yaw - current_yaw > M_PI)    diff_yaw = 2 * M_PI - (start_yaw - current_yaw);
                else if (current_yaw > start_yaw)           diff_yaw = current_yaw - start_yaw; 
                else                                        diff_yaw = start_yaw - current_yaw;

                if (diff_yaw > M_PI_4)
                {
                    px4_interface_->set_pos(last_safe_position_[0], last_safe_position_[1], last_safe_position_[2], start_yaw);
                }
                else
                {
                    motion_manager_->resetMotion();
                    last_is_rotation_ = false;
                }
            }
            else
            {
                mode_mutex_.lock();
                motion_manager_->localPlanning();
                mode_mutex_.unlock();
            }
        }
        // if (motion_mode_ != MOTION_MODE::MOVING)
        // {
        //     motion_manager_->visCleaner();
        // }
    }

    Eigen::Vector3d ExploreManager::collisionDetection(Eigen::Vector3d &target_pos)
    {
        int sample_num = 5;
        double map_resolution = map_manager_->map_->getResolution();

        for (int x = -sample_num; x < sample_num; ++x)
        {
            for (int y = -sample_num; y < sample_num; ++y)
            {
                Eigen::Vector3d tmp_p = target_pos + Eigen::Vector3d(x * map_resolution, y * map_resolution, 0.0);

                if (map_manager_->map_->getInflateOccupancy(tmp_p)) {
                    return last_safe_position_; // 如果目标点本身就有碰撞，返回上次的安全位置
                }
            }
        }

        return target_pos; // 如果没有碰撞，返回原目标位置
    }

    void ExploreManager::calcNextTargetFrontier(Eigen::Vector3d start_pos)
    {
        map_manager_->frontier_->updateCost(start_pos, hovering_yaw_);

        calc_nbp_ing_ = false;
    }


    void ExploreManager::getTargetFrontierInfo(Eigen::Vector3d &point, Eigen::Vector2d &normal)
    {
        map_manager_->frontier_->getBestNextFrontierInfo(point, normal);
    }


    int ExploreManager::getFrontiersNumber()
    {
        return map_manager_->frontier_->getFrontiersNumber();
    }

    void ExploreManager::change2Mode(MOTION_MODE mode)
    {
        motion_mode_ = mode;
    }
}
