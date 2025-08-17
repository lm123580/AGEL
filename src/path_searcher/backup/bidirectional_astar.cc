#include "astar.h"
#include "sdf_map.h"

namespace AGEL
{
    Astar::Astar(ros::NodeHandle &nh, const std::shared_ptr<SDFMap> &map)
    {
        nh.param("astar/resolution_astar", resolution_, 0.40);
        nh.param("astar/lambda_heu", lambda_heu_, 1.5);
        nh.param("astar/max_search_time", max_search_time_, 0.05);
        nh.param("astar/allocate_num", allocate_num_, 1000000);

        tie_breaker_ = 1.0 + 1e-3;

        sdf_map_ = map;

        inv_resolution_ = 1.0 / resolution_;
        sdf_map_->getRegion(origin_, map_size_3d_);

        path_node_pool_.resize(allocate_num_);
        for (int i = 0; i < allocate_num_; i++) 
        {
            path_node_pool_[i].reset(new Node);
        }

        use_node_num_ = 0;

        path_inflate_points_.emplace_back(-4.0, -4.0, 0.0);
        path_inflate_points_.emplace_back(0.0, -4.0, 0.0);
        path_inflate_points_.emplace_back(4.0, -4.0, 0.0);
        path_inflate_points_.emplace_back(-2.0, -2.0, 0.0);
        path_inflate_points_.emplace_back(0.0, -2.0, 0.0);
        path_inflate_points_.emplace_back(2.0, -2.0, 0.0);
        path_inflate_points_.emplace_back(-4.0, 0.0, 0.0);
        path_inflate_points_.emplace_back(-2.0, 0.0, 0.0);
        path_inflate_points_.emplace_back(2.0, 0.0, 0.0);
        path_inflate_points_.emplace_back(4.0, 0.0, 0.0);
        path_inflate_points_.emplace_back(-2.0, 2.0, 0.0);
        path_inflate_points_.emplace_back(0.0, 2.0, 0.0);
        path_inflate_points_.emplace_back(2.0, 2.0, 0.0);
        path_inflate_points_.emplace_back(-4.0, 4.0, 0.0);
        path_inflate_points_.emplace_back(0.0, 4.0, 0.0);
        path_inflate_points_.emplace_back(4.0, 4.0, 0.0);

        path_inflate_points_.emplace_back(-3.0, 0.0, 0.0);
        path_inflate_points_.emplace_back(3.0, 0.0, 0.0);
        path_inflate_points_.emplace_back(-1.0, 0.0, 0.0);
        path_inflate_points_.emplace_back(1.0, 0.0, 0.0);
        path_inflate_points_.emplace_back(0.0, -3.0, 0.0);
        path_inflate_points_.emplace_back(0.0, 3.0, 0.0);
        path_inflate_points_.emplace_back(0.0, -1.0, 0.0);
        path_inflate_points_.emplace_back(0.0, 1.0, 0.0);
        for (auto &path_pt : path_inflate_points_) 
        {
            path_pt *= (sdf_map_->getResolution() * 0.5);
        }
    }

    Astar::~Astar() = default;

    int Astar::search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) 
    {
        reset();

        start_pt_ = start_pt;
        end_pt_ = end_pt;

        if ((start_pt - end_pt).norm() < resolution_)
        {
            search_path_nodes_.push_back(start_pt);
            search_path_nodes_.push_back(end_pt);

            return REACH_END;
        }

        // 初始化起始节点和终止节点
        NodePtr start_node = path_node_pool_[0];
        start_node->parent = nullptr;
        start_node->position = start_pt;
        posToIndex(start_pt, start_node->index);
        start_node->g_score = 0.0;
        start_node->f_score = lambda_heu_ * getDiagHeu(start_node->position, end_pt);

        NodePtr end_node = path_node_pool_[1];
        end_node->parent = nullptr;
        end_node->position = end_pt;
        posToIndex(end_pt, end_node->index);
        end_node->g_score = 0.0;
        end_node->f_score = lambda_heu_ * getDiagHeu(end_node->position, start_pt);

        // 将初始节点和终止节点分别加入两个open set
        open_set_start_.push(start_node);
        open_set_map_start_.insert(make_pair(start_node->index, start_node));

        open_set_end_.push(end_node);
        open_set_map_end_.insert(make_pair(end_node->index, end_node));

        use_node_num_ += 2;

        const auto t1 = ros::Time::now();
        
        while (!open_set_start_.empty() && !open_set_end_.empty()) 
        {
            if (searchOneDirection(open_set_start_, open_set_map_start_, close_set_map_start_, open_set_map_end_, true)) 
            {
                return REACH_END;
            }

            if (searchOneDirection(open_set_end_, open_set_map_end_, close_set_map_end_, open_set_map_start_, false)) 
            {
                return REACH_END;
            }

            if ((ros::Time::now() - t1).toSec() > max_search_time_) 
            {
                ROS_WARN("Astar: Search timeout!");
                return NO_PATH;
            }
        }

        std::cout << "open set empty, no path!" << std::endl;
        std::cout << "use node num: " << use_node_num_ << std::endl;

        return NO_PATH;
    }

    int Astar::globalSearch(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt)
    {
        if (search(start_pt, end_pt) == NO_PATH)
        {
            return NO_PATH;
        }

        global_path_nodes_ = std::move(search_path_nodes_);

        double length = 0.0;
        int global_path_size = global_path_nodes_.size();

        local_path_nodes_.clear();
        local_path_nodes_.push_back(global_path_nodes_.front());
        for (int i = 1; i < global_path_size && length < 5.0; i++)
        {
            length += (global_path_nodes_[i] - global_path_nodes_[i - 1]).norm();
            local_path_nodes_.push_back(global_path_nodes_[i]);
        }

        
        return REACH_END;
    }


    // int Astar::reSearch(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) 
    // {
    //     local_path_nodes_.clear();

    //     // 遍历以获得距离起点最近路径点
    //     int min_node_index = 0;
    //     int global_size = global_path_nodes_.size();
    //     double min_dist = std::numeric_limits<double>::max();
    //     for (int i = 0; i < global_size; i++)
    //     {
    //         double dist = (global_path_nodes_[i] - start_pt).squaredNorm();
    //         if (dist < min_dist)
    //         {
    //             min_dist = dist;
    //             min_node_index = i;
    //         }
    //     }

    //     // 如果最近路径点是终点，那么直接返回
    //     if (min_node_index == global_size - 1)
    //     {
    //         if (global_size >= 2)
    //         {
    //             local_path_nodes_ = global_path_nodes_;
    //         }
    //         else
    //         {
    //             local_path_nodes_.push_back(start_pt);
    //             local_path_nodes_.push_back(global_path_nodes_[0]);
    //             global_path_nodes_ = local_path_nodes_;

    //             return CHANGE_END;
    //         }

    //         return REACH_END;
    //     }

    //     // 删除已经经过的路径点
    //     std::vector<Eigen::Vector3d> tmp_path_nodes;
    //     tmp_path_nodes.insert(tmp_path_nodes.begin(), global_path_nodes_.begin() + min_node_index, global_path_nodes_.end());

    //     // 判断路径上是否存在障碍物
    //     int start_index = -1;
    //     int tmp_path_size = tmp_path_nodes.size();
    //     Eigen::Vector3d tmp_pt;
    //     bool safe = true;
    //     for (int i = 0; i < tmp_path_size && safe; i++)
    //     {
    //         for (const auto &path_pt: path_inflate_points_)
    //         {
    //             tmp_pt = tmp_path_nodes[i] + path_pt;
    //             if (sdf_map_->getInflateOccupancy(tmp_pt) == true)
    //             {
    //                 start_index = i;
    //                 safe = false;
    //                 break;
    //             }                
    //         }
    //     }

    //     // 如果路径上不存在障碍物
    //     int global_path_size;

    //     if (start_index == -1)
    //     {
    //         global_path_nodes_ = tmp_path_nodes;
    //     }
    //     else
    //     {
    //         Eigen::Vector3d global_start_node = tmp_path_nodes[0];

    //         global_path_nodes_.clear();

    //         int end_index = tmp_path_size;
    //         std::vector<Eigen::Vector3d> start_path, mid_path, end_path;
            
    //         for (int i = start_index + 1; i < tmp_path_size; i++)
    //         {
    //             safe = true;
    //             for (const auto &path_pt: path_inflate_points_)
    //             {
    //                 tmp_pt = tmp_path_nodes[i] + path_pt;
    //                 if (sdf_map_->getInflateOccupancy(tmp_pt) == true)
    //                 {
    //                     // start_index = i;
    //                     safe = false;
    //                     break;
    //                 }                
    //             }

    //             if (safe)
    //             {
    //                 end_index = i;
    //                 break;
    //             }
    //         }
            
            
    //         if (start_index >= 2)
    //         {
    //             start_path.insert(start_path.begin(), tmp_path_nodes.begin(), tmp_path_nodes.begin() + start_index - 2);
    //         }

    //         if (end_index != tmp_path_size)
    //         {
    //             int search_result;
    //             if (start_index == 0)
    //             {
    //                 search_result = search(start_pt, tmp_path_nodes[end_index]);
    //             }
    //             else
    //             {
    //                 search_result = search(tmp_path_nodes[start_index - 1], tmp_path_nodes[end_index]);
    //             }

    //             if (search_result == REACH_END)
    //             {
    //                 mid_path = std::move(search_path_nodes_);
    //             }
    //         }

    //         if (end_index + 1 < tmp_path_size)
    //         {
    //             end_path.insert(end_path.begin(), tmp_path_nodes.begin() + end_index + 1, tmp_path_nodes.end());
    //         }

    //         int start_path_size = start_path.size();
    //         int mid_path_size = mid_path.size();
    //         if (start_path_size + mid_path_size >= 2)
    //         {
    //             global_path_nodes_.insert(global_path_nodes_.end(), start_path.begin(), start_path.end());
    //             global_path_nodes_.insert(global_path_nodes_.end(), mid_path.begin(), mid_path.end());
    //             global_path_nodes_.insert(global_path_nodes_.end(), end_path.begin(), end_path.end());                
    //         }

    //         global_path_size = global_path_nodes_.size();
    //         if (global_path_size < 2)
    //         {
    //             global_path_nodes_.push_back(start_pt);
    //             global_path_nodes_.push_back(global_start_node);
    //         }
    //     }

    //     double length = 0.0;
    //     global_path_size = global_path_nodes_.size();
    //     local_path_nodes_.push_back(global_path_nodes_.front());
    //     for (int i = 1; i < global_path_size && length < 5.0; i++)
    //     {
    //         length += (global_path_nodes_[i] - global_path_nodes_[i - 1]).norm();
    //         local_path_nodes_.push_back(global_path_nodes_[i]);
    //     }

    //     if ((global_path_nodes_.back() - end_pt).norm() > 1e-3)
    //     {
    //         std::cout << global_path_nodes_.size() << std::endl;
    //         std::cout << "change world" << std::endl;
    //         return CHANGE_END;
    //     }
    //     else
    //     {
    //         return REACH_END;
    //     }
    // }


    int Astar::reSearch(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) 
    {
        local_path_nodes_.clear();

        // 遍历以获得距离起点最近路径点
        int min_node_index = 0;
        int global_size = global_path_nodes_.size();
        double min_dist = std::numeric_limits<double>::max();
        for (int i = 0; i < global_size; i++)
        {
            double dist = (global_path_nodes_[i] - start_pt).squaredNorm();
            if (dist < min_dist)
            {
                min_dist = dist;
                min_node_index = i;
            }
        }

        // 如果最近路径点是终点，那么直接返回
        if (min_node_index == global_size - 1)
        {
            if (global_size >= 2)
            {
                local_path_nodes_ = global_path_nodes_;
            }
            else
            {
                local_path_nodes_.push_back(start_pt);
                local_path_nodes_.push_back(global_path_nodes_[0]);
                global_path_nodes_ = local_path_nodes_;

                return CHANGE_END;
            }

            return REACH_END;
        }

        // 删除已经经过的路径点
        std::vector<Eigen::Vector3d> tmp_path_nodes;
        tmp_path_nodes.insert(tmp_path_nodes.begin(), global_path_nodes_.begin() + min_node_index, global_path_nodes_.end());

        // 判断路径上是否存在障碍物
        int start_index = -1;
        int tmp_path_size = tmp_path_nodes.size();
        Eigen::Vector3d tmp_pt;
        bool safe = true;
        for (int i = 0; i < tmp_path_size && safe; i++)
        {
            for (const auto &path_pt: path_inflate_points_)
            {
                tmp_pt = tmp_path_nodes[i] + path_pt;
                if (sdf_map_->getInflateOccupancy(tmp_pt) == true)
                {
                    start_index = i;
                    safe = false;
                    break;
                }                
            }
        }

        // 如果路径上不存在障碍物
        int global_path_size;

        if (start_index == -1)
        {
            global_path_nodes_ = tmp_path_nodes;
        }
        else
        {
            // Eigen::Vector3d global_start_node = tmp_path_nodes[0];
            global_path_nodes_.clear();

            int end_index;
            // std::vector<Eigen::Vector3d> start_path, mid_path, end_path;
            
            for (int i = tmp_path_size - 1; i > start_index; i--)
            {
                safe = true;
                for (const auto &path_pt: path_inflate_points_)
                {
                    tmp_pt = tmp_path_nodes[i] + path_pt;
                    if (sdf_map_->getInflateOccupancy(tmp_pt) == true)
                    {
                        // start_index = i;
                        safe = false;
                        break;
                    }                
                }

                if (safe)
                {
                    end_index = i;
                    break;
                }
            }

            int search_result = search(start_pt, tmp_path_nodes[end_index]);

            if (search_result == REACH_END)
            {
                global_path_nodes_ = std::move(search_path_nodes_);
            }

            global_path_size = global_path_nodes_.size();
            if (global_path_size < 2)
            {
                global_path_nodes_.push_back(start_pt);
                global_path_nodes_.push_back(tmp_path_nodes.front());
            }
        }

        double length = 0.0;
        global_path_size = global_path_nodes_.size();
        local_path_nodes_.push_back(global_path_nodes_.front());
        for (int i = 1; i < global_path_size && length < 5.0; i++)
        {
            length += (global_path_nodes_[i] - global_path_nodes_[i - 1]).norm();
            local_path_nodes_.push_back(global_path_nodes_[i]);
        }

        if ((global_path_nodes_.back() - end_pt).norm() > 1e-3)
        {
            std::cout << global_path_nodes_.size() << std::endl;
            std::cout << "change world" << std::endl;
            return CHANGE_END;
        }
        else
        {
            return REACH_END;
        }
    }

    bool Astar::searchOneDirection(
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> &open_set,
        std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> &open_set_map,
        std::unordered_map<Eigen::Vector3i, int, matrix_hash<Eigen::Vector3i>> &close_set_map,
        std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> &other_open_set_map,
        bool forward)
    {
        NodePtr cur_node = open_set.top();
        open_set.pop();

        if (other_open_set_map.find(cur_node->index) != other_open_set_map.end()) 
        {
            NodePtr meeting_node = other_open_set_map[cur_node->index];
            if (forward) 
            {
                backtrack(cur_node, meeting_node);
            } 
            else 
            {
                backtrack(meeting_node, cur_node);
            }
            return true;
        }

        open_set_map.erase(cur_node->index);
        close_set_map.insert(std::make_pair(cur_node->index, 1));

        Eigen::Vector3d cur_pos = cur_node->position;
        Eigen::Vector3d nbr_pos;
        Eigen::Vector3d step;

        for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
        {    
            for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
            {
                for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) 
                {
                    step << dx, dy, dz;
                    if (step.norm() < 1e-3) continue;
                    nbr_pos = cur_pos + step;
                    
                    if (sdf_map_->getInflateOccupancy(nbr_pos) == true) continue;

                    bool safe = true;
                    for (const auto &path_pt : path_inflate_points_)
                    {
                        Eigen::Vector3d ckpt = nbr_pos + path_pt;
                        if (sdf_map_->getInflateOccupancy(ckpt) == true)
                        {
                            safe = false;
                            break;
                        }
                    }
                    if (!safe) continue;
                    
                    Eigen::Vector3i nbr_idx;
                    posToIndex(nbr_pos, nbr_idx);
                    if (close_set_map.find(nbr_idx) != close_set_map.end()) continue;

                    NodePtr neighbor;
                    double tmp_g_score = step.norm() + cur_node->g_score;

                    auto node_iter = open_set_map.find(nbr_idx);

                    if (node_iter == open_set_map.end()) 
                    {
                        neighbor = path_node_pool_[use_node_num_];
                        use_node_num_ += 1;
                        if (use_node_num_ == allocate_num_) 
                        {
                            ROS_WARN("Astar: Run out of node pool!");
                            return false;
                        }
                        neighbor->index = nbr_idx;
                        neighbor->position = nbr_pos;
                    } 
                    else if (tmp_g_score < node_iter->second->g_score) 
                    {
                        neighbor = node_iter->second;
                    } 
                    else 
                    {
                        continue;
                    }

                    neighbor->parent = cur_node;
                    neighbor->g_score = tmp_g_score;
                    neighbor->f_score = tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, forward ? end_pt_ : start_pt_);
                    open_set.push(neighbor);
                    open_set_map[nbr_idx] = neighbor;
                }
            }
        }
        return false;
    }

    void Astar::setResolution(const double &res) 
    {
        resolution_ = res;
        inv_resolution_ = 1.0 / resolution_;
    }

    void Astar::reset() 
    {
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> empty_queue_start, empty_queue_end;
        std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> empty_open_map_start, empty_open_map_end;
        std::unordered_map<Eigen::Vector3i, int, matrix_hash<Eigen::Vector3i>> empty_close_map_start, empty_close_map_end;
        std::vector<Eigen::Vector3d> empty_path;

        open_set_start_.swap(empty_queue_start);
        open_set_map_start_.swap(empty_open_map_start);
        close_set_map_start_.swap(empty_close_map_start);

        open_set_end_.swap(empty_queue_end);
        open_set_map_end_.swap(empty_open_map_end);
        close_set_map_end_.swap(empty_close_map_end);

        search_path_nodes_.swap(empty_path);

        use_node_num_ = 0;
    }

    void Astar::backtrack(NodePtr meeting_node_start, NodePtr meeting_node_end)
    {
        std::vector<Eigen::Vector3d> path_from_start, path_from_end;

        // 从meeting_node_start回溯到起点
        NodePtr cur_node = meeting_node_start;
        while (cur_node != nullptr)
        {
            path_from_start.push_back(cur_node->position);
            cur_node = cur_node->parent;
        }

        // 从meeting_node_end回溯到终点
        cur_node = meeting_node_end;
        while (cur_node != nullptr)
        {
            path_from_end.push_back(cur_node->position);
            cur_node = cur_node->parent;
        }

        // 连接两部分路径
        std::reverse(path_from_start.begin(), path_from_start.end());
        search_path_nodes_ = path_from_start;

        // 避免重复添加meeting_node_end的position
        search_path_nodes_.insert(search_path_nodes_.end(), path_from_end.begin() + 1, path_from_end.end());
    }


    void Astar::getGlobalPath(std::vector<Eigen::Vector3d> &path) 
    {
        path = global_path_nodes_;
    }

    void Astar::getLocalPath(std::vector<Eigen::Vector3d> &path) 
    {
        path = local_path_nodes_;
    }

    double Astar::pathLength(std::vector<Eigen::Vector3d> &path)
    {
        double length = 0.0;

        if (path.size() < 2)
        {
            std::cout << "Path Only 1 Point!!!" << std::endl;

            return length;
        } 

        int path_size = path.size() - 1;
        for (int i = 0; i < path_size; i++)
        {
            length += (path[i+1] - path[i]).norm();
        }

        return length;
    }

    double Astar::getDiagHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2) 
    {
        double dx = fabs(x1(0) - x2(0));
        double dy = fabs(x1(1) - x2(1));
        double dz = fabs(x1(2) - x2(2));
        double h  = 0;
        double diag = std::min(std::min(dx, dy), dz);

        dx -= diag;
        dy -= diag;
        dz -= diag;

        if (dx < 1e-4) h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dy, dz) + 1.0 * abs(dy - dz);
        if (dy < 1e-4) h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dz) + 1.0 * abs(dx - dz);
        if (dz < 1e-4) h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * abs(dx - dy);

        return tie_breaker_ * h;
    }

    void Astar::posToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) 
    {
        idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
    }
}
