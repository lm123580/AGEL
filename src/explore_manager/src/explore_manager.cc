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
        nh.param("camera_util/max_dist", max_ray_length_, 4.5);
        nh.param("explore_manager/delta_yaw", delta_yaw_, 0.19635);

        map_manager_ = map_manager;
        px4_interface_ = px4_interface;

        motion_manager_.reset(new MotionManager());

        motion_manager_->init(nh, map_manager_, px4_interface_);

        control_tiemr_ = nh.createTimer(ros::Duration(0.020), &ExploreManager::controlCallback,this);
    }


    void ExploreManager::run()
    {
        switch (motion_mode_)
        {
        case MOTION_MODE::INIT:
        {
            // 初始化
            hovering_yaw_ = px4_interface_->get_yaw();
            hovering_pose_ = px4_interface_->get_pos();
            change2Mode(MOTION_MODE::ROTATION);

            break;
        }

        case MOTION_MODE::ROTATION:
        {
            // 旋转采样
            double current_yaw;
            
            hovering_pose_ = px4_interface_->get_pos();
            hovering_pose_[2] = 1.20;
            current_yaw = px4_interface_->get_yaw();
            
            getRotationDirection(hovering_pose_, current_yaw);
            
            if (rotation_direction_ == ROTATION_DIRECTION::NONE)
            {
                calc_nbp_start_ = false;
                calc_nbp_ing_ = false;
                hovering_yaw_ = current_yaw;
                change2Mode(MOTION_MODE::HOVER);
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
                    getTargetFrontierInfo(target_frontier_point_, target_frontier_normal_);

                    if (getFrontiersNumber() == 0)
                    {
                        Eigen::Vector3d current_pos = px4_interface_->get_pos();
                        if (!map_manager_->map_->getInflateOccupancy(current_pos))
                        {
                            // 没有前沿点了，结束探索
                            change2Mode(MOTION_MODE::FINISH);
                        }
                        else
                        {
                            // 没有前沿点了，重新寻找下一个目标前沿点
                            calc_nbp_start_ = false;
                            calc_nbp_ing_ = false;
                        }
                        
                        return ;
                    }

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
                std::thread t_calc(&ExploreManager::calcNextTargetFrontier, this);
                t_calc.detach();
            }

            break;
        }

        case MOTION_MODE::MOVING:
        {
            if (frontierExplored(target_frontier_point_, target_frontier_normal_))
            {
                hovering_pose_ = px4_interface_->get_pos();
                hovering_yaw_ = px4_interface_->get_yaw();
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
            hovering_yaw_ = px4_interface_->get_yaw();
            hovering_pose_ = px4_interface_->get_pos();

            ROS_INFO_THROTTLE(1.0, "<<<<<===============FINISH!!!==================>>>>>");

            break;
        }
        
        default:
            break;
        }
    }


    void ExploreManager::getRotationDirection(Eigen::Vector3d& current_pos, double& current_yaw)
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
                    if (rotation_direction_ == ROTATION_DIRECTION::NONE)
                    {
                        rotation_direction_ = ROTATION_DIRECTION::CLOCKWISE;
                    }

                    return ;
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
                    if (rotation_direction_ == ROTATION_DIRECTION::NONE)
                    {
                        rotation_direction_ = ROTATION_DIRECTION::COUNTERCLOCKWISE;
                    }

                    return ;
                }
            }

            // 增量
            tmp_yaw += delta_yaw;
        }

        rotation_direction_ = ROTATION_DIRECTION::NONE;
    }


    bool ExploreManager::frontierExplored(Eigen::Vector3d& point, Eigen::Vector2d& normal)
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

        double height = 1.20;
        double current_yaw = px4_interface_->get_yaw();
        Eigen::Vector3d current_pos = px4_interface_->get_pos();
        current_pos[2] = height;
        if (map_manager_->map_->getInflateOccupancy(current_pos))
        {
            current_pos = collisionDetection(current_pos);
            hovering_pose_ = current_pos;
            px4_interface_->set_pos(current_pos[0], current_pos[1], height, current_yaw);
            return ;
        }


        switch (motion_mode_)
        {
        case MOTION_MODE::INIT:
        {
            return ;
        }

        case MOTION_MODE::ROTATION:
        {
            if (rotation_direction_ == ROTATION_DIRECTION::CLOCKWISE)
            {
                double yaw_rate = 1.57;
                px4_interface_->set_pos_with_yaw_rate(hovering_pose_(0), hovering_pose_(1), hovering_pose_(2), yaw_rate);
            }
            else if (rotation_direction_ == ROTATION_DIRECTION::COUNTERCLOCKWISE)
            {
                double yaw_rate = -1.57;
                px4_interface_->set_pos_with_yaw_rate(hovering_pose_(0), hovering_pose_(1), hovering_pose_(2), yaw_rate);
            }

            break;
        }

        case MOTION_MODE::HOVER:
        {
            px4_interface_->set_pos(hovering_pose_(0), hovering_pose_(1), hovering_pose_(2), hovering_yaw_);

            break;
        }

        case MOTION_MODE::MOVING:
        {
            if (last_is_rotation_)
            {
                double start_yaw = motion_manager_->getGlobalStartYaw();

                if (start_yaw > 100.0)
                {
                    px4_interface_->set_pos(current_pos[0], current_pos[1], height, current_yaw);

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
                mode_mutex_.lock();
                motion_manager_->localPlanning();
                mode_mutex_.unlock();
            }

            break;
        }
        default:
            break;
        }

        if (motion_mode_ != MOTION_MODE::MOVING)
        {
            motion_manager_->visCleaner();
        }
    }
    Eigen::Vector3d ExploreManager::collisionDetection(Eigen::Vector3d &target_pos)
    {
        int sample_num = 5;
        double map_resolution = map_manager_->map_->getResolution();

        std::vector<Eigen::Vector3d> collision_points;
        std::vector<Eigen::Vector3d> safe_points;

        for (int x = -sample_num; x < sample_num; ++x)
        {
            for (int y = -sample_num; y < sample_num; ++y)
            {
                Eigen::Vector3d tmp_p = target_pos + Eigen::Vector3d(x * map_resolution, y * map_resolution, 0.0);

                if (map_manager_->map_->getInflateOccupancy(tmp_p))
                    collision_points.push_back(tmp_p);
                else
                    safe_points.push_back(tmp_p);
            }
        }

        // 目标点本身安全，直接返回
        if (collision_points.empty())
            return target_pos;

        // 没有安全点，返回原点或自定义异常处理
        if (safe_points.empty())
            return target_pos; // 或者 throw std::runtime_error("No safe point found!");

        // 选离所有碰撞点最远的安全点
        Eigen::Vector3d best_point = safe_points[0];
        double max_dist = -std::numeric_limits<double>::max();
        for (const auto& sp : safe_points)
        {
            double dist_sum = 0.0;
            for (const auto& cp : collision_points)
                dist_sum += (sp - cp).squaredNorm();

            if (dist_sum > max_dist)
            {
                max_dist = dist_sum;
                best_point = sp;
            }
        }

        return best_point;
    }

    // Eigen::Vector3d ExploreManager::collisionDetection(Eigen::Vector3d &target_pos)
    // {
    //     int sample_num = 5;
    //     std::vector<bool> collision_flag;
    //     std::vector<Eigen::Vector3d> sample_points, collision_points;

    //     double map_resolution = map_manager_->map_->getResolution();

    //     Eigen::Vector3d tmp_p;
    //     for (int x = -sample_num; x < sample_num; x++)
    //     {
    //         for (int y = -sample_num; y < sample_num; y++)
    //         {
    //             tmp_p = Eigen::Vector3d(x * map_resolution, y * map_resolution, 0.0) + target_pos;
    //             sample_points.push_back(tmp_p);
    //         }
    //     }

    //     int sample_size = sample_points.size();
    //     for (int i = 0; i < sample_size; i++)
    //     {
    //         tmp_p = sample_points[i];

    //         if (map_manager_->map_->getInflateOccupancy(tmp_p))
    //         {
    //             collision_flag.push_back(true);
    //             collision_points.push_back(tmp_p);
    //         }
    //         else
    //         {
    //             collision_flag.push_back(false);
    //         }
    //     }

    //     if (collision_points.size() == 0)
    //     {
    //         return target_pos;
    //     }

    //     Eigen::Vector3d safe_pos;
    //     double max_dist = -std::numeric_limits<double>::max();
    //     for (int i = 0; i < sample_size; i++)
    //     {
    //         if (collision_flag[i])  continue ;

    //         double tmp_dist = 0.0;
    //         for (auto &cp: collision_points)
    //         {
    //             tmp_dist += (sample_points[i] - cp).squaredNorm();
    //         }

    //         if (tmp_dist > max_dist)
    //         {
    //             max_dist = tmp_dist;
    //             safe_pos = sample_points[i];
    //         }
    //     }

    //     return safe_pos;
    // }


    void ExploreManager::calcNextTargetFrontier()
    {   
        while (map_manager_->map_->getInflateOccupancy(hovering_pose_))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        map_manager_->frontier_->updateCost(hovering_pose_, hovering_yaw_);

        
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
