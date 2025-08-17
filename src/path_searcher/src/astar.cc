#include "astar.h"
#include "sdf_map.h"

namespace AGEL
{
    Astar::Astar(ros::NodeHandle &nh, const std::shared_ptr<SDFMap> &map)
    {
        nh.param("astar/resolution_astar", resolution_, 0.4);
        nh.param("astar/lambda_heu", lambda_heu_, 1.0);
        nh.param("astar/max_search_time", max_search_time_, 0.010);
        nh.param("astar/allocate_num", allocate_num_, 1000000);

        tie_breaker_ = 1.0 + 1e-3;

        map_ = map;

        inv_resolution_ = 1.0 / resolution_;
        map->getRegion(origin_, map_size_3d_);

        map_resolution_ = map->getResolution();
        check_occ_step_ = std::ceil((resolution_ / map_resolution_) * 1.5);

        path_node_pool_.resize(allocate_num_);
        for (int i = 0; i < allocate_num_; i++) 
        {
            path_node_pool_[i].reset(new Node);
        }

        use_node_num_ = 0;

        Eigen::Vector3d step;
        search_neighbors_.clear();
        for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
        {    
            for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
            {
                for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) 
                {
                    step << dx, dy, dz;

                    if (step.norm() < 1e-3) continue;

                    search_neighbors_.emplace_back(step);
                    search_step_length_.emplace_back(step.norm());
                }
            }
        }

        // iter_num_ = 0;
    }


    Astar::~Astar() = default;

    bool Astar::globalPlan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt)
    {
        return globalPlan(start_pt, end_pt, max_search_time_);
    }


    bool Astar::globalPlan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt, const double &time_limit)
    {
        if (search(start_pt, end_pt, time_limit))
        {
            global_path_nodes_ = std::move(search_path_nodes_);

            path_nodes_ = global_path_nodes_;

            return true;
        }

        return false;
    }


    bool Astar::replan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt)
    {
        path_nodes_.clear();

        // 寻找最近的点
        int min_idx = 0;
        int global_path_size = global_path_nodes_.size();
        
        double sqdis;
        double min_sqdis = (global_path_nodes_[0] - start_pt).squaredNorm();
        for (int i = 1; i < global_path_size; i++)
        {
            sqdis = (global_path_nodes_[i] - start_pt).squaredNorm();
            if (sqdis < min_sqdis)
            {
                min_sqdis = sqdis;
                min_idx = i;
            }
        }

        // 得到局部路径点
        path_nodes_.emplace_back(global_path_nodes_[min_idx]);
        for (int i = min_idx + 1; i < global_path_size; i++)
        {
            if (isOccupied(path_nodes_.back(), global_path_nodes_[i]))
            {
                return false;
            }

            path_nodes_.emplace_back(global_path_nodes_[i]);
        }

        // 保证至少有2个路径点
        if (path_nodes_.size() < 2)
        {
            if (!isOccupied(start_pt, end_pt) && (start_pt - end_pt).norm() < resolution_ + 1e-3)
            {
                path_nodes_.push_back(start_pt);
                path_nodes_.push_back(end_pt);
                
                return true;
            }
            else
            {
                return false;
            }
        }

        return true;
    }


    // bool Astar::search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt)
    // {
    //     return search(start_pt, end_pt, max_search_time_);
    // }


    bool Astar::search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt, const double &time_limit)
    {
        reset();
        
        // 初始节点
        NodePtr cur_node = path_node_pool_[use_node_num_++];
        cur_node->parent = nullptr;
        cur_node->position = start_pt;
        posToIndex(start_pt, cur_node->index);
        cur_node->g_score = 0.0;
        cur_node->f_score = lambda_heu_ * getDiagHeu(cur_node->position, end_pt);

        // 终止节点
        Eigen::Vector3i end_index;
        posToIndex(end_pt, end_index);

        // 将初始节点加入open set
        open_queue_.push(cur_node);
        open_map_set_[cur_node->index] = cur_node;

        ros::Time t1 = ros::Time::now();
        
        // 开始路径搜索
        while ((!open_queue_.empty())) 
        {
            cur_node = open_queue_.top();
            bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                             abs(cur_node->index(1) - end_index(1)) <= 1 && 
                             abs(cur_node->index(2) - end_index(2)) <= 1;
            
            // 找到终点
            if (reach_end) 
            {
                backtrack(cur_node, end_pt);

                return true;
            }

            // 超过搜索时间
            if ((ros::Time::now() - t1).toSec() > time_limit) 
            {
                // ROS_WARN("Astar: Search timeout!");

                return false;
            }

            // 从open set中删除当前节点，并放入close set
            open_queue_.pop();
            open_map_set_.erase(cur_node->index);
            close_set_.insert(cur_node->index);

            Eigen::Vector3d cur_pos = cur_node->position;

            int nbr_size = search_neighbors_.size();
            double nbr_length;
            Eigen::Vector3i nbr_idx;
            Eigen::Vector3d nbr_pos, nbr_step;
            for (int i = 0; i < nbr_size; i++)
            {
                nbr_step = search_neighbors_[i];
                nbr_length = search_step_length_[i];

                nbr_pos = cur_pos + nbr_step;
                if (isOccupied(cur_pos, nbr_pos))   continue;

                // 检查节点是否在闭集中
                posToIndex(nbr_pos, nbr_idx);
                if (close_set_.find(nbr_idx) != close_set_.end()) continue;

                NodePtr neighbor;
                double tmp_g_score = nbr_length + cur_node->g_score;           // 到邻居节点的代价

                auto nbr_iter = open_map_set_.find(nbr_idx);
                if (nbr_iter == open_map_set_.end())                                // 当邻居节点不在open set中
                {
                    neighbor = path_node_pool_[use_node_num_++];                    // 分配一个新的内存
                    
                    if (use_node_num_ == allocate_num_) 
                    {
                        // ROS_WARN("Astar: Run out of node pool!");

                        return false;
                    }

                    neighbor->index = nbr_idx;
                    neighbor->position = nbr_pos;
                } 
                else if (tmp_g_score < nbr_iter->second->g_score) neighbor = nbr_iter->second; // 更新
                else continue;
                        
                // 加入或更新邻居节点
                neighbor->parent = cur_node;
                neighbor->g_score = tmp_g_score;
                neighbor->f_score = tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, end_pt);
                open_queue_.push(neighbor);
                open_map_set_[nbr_idx] = neighbor;
            }
        }

        // std::cout << "open set empty, no path!" << std::endl;
        // std::cout << "use node num: " << use_node_num_ << std::endl;

        return false;
    }


    void Astar::setResolution(const double &res) 
    {
        resolution_ = res;
        inv_resolution_ = 1.0 / resolution_;
    }


    bool Astar::isOccupied(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2)
    {
        Eigen::Vector3d dir = pos2 - pos1;
        double step_length = dir.norm() / (double)check_occ_step_;
        Eigen::Vector3d update_step = dir.normalized() * step_length;

        Eigen::Vector3d cur_pos = pos1;
        for (int i = 0; i <= check_occ_step_; i++)
        {
            if (map_->getInflateOccupancy(cur_pos) || map_->getOccupancy(cur_pos) == map_->UNKNOWN)
            {
                return true;
            }

            cur_pos += update_step;
        }

        return false;
    }


    void Astar::reset() 
    {
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> empty_queue;
        std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> empty_map_set;
        std::unordered_set<Eigen::Vector3i, matrix_hash<Eigen::Vector3i>> empty_set;
        std::vector<Eigen::Vector3d> empty_path;

        open_queue_.swap(empty_queue);
        open_map_set_.swap(empty_map_set);
        close_set_.swap(empty_set);
        search_path_nodes_.swap(empty_path);

        use_node_num_ = 0;
    }


    void Astar::backtrack(const NodePtr &end_node, const Eigen::Vector3d &end) 
    {
        search_path_nodes_.push_back(end);
        search_path_nodes_.push_back(end_node->position);
        NodePtr cur_node = end_node;
       
        while (cur_node->parent != nullptr) 
        {
            cur_node = cur_node->parent;
            search_path_nodes_.push_back(cur_node->position);
        }

        std::reverse(search_path_nodes_.begin(), search_path_nodes_.end());
    }


    void Astar::getPath(std::vector<Eigen::Vector3d> &path) 
    {
        path = path_nodes_;
    }

    double Astar::pathLength()
    {
        double length = 0.0;

        if (path_nodes_.size() < 2) return length;

        int path_size = path_nodes_.size() - 1;
        for (int i = 0; i < path_size; i++)
        {
            length += (path_nodes_[i+1] - path_nodes_[i]).norm();
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

        // sqrt(3.0) = 1.7320508075688773
        // sqrt(2.0) = 1.4142135623730950
        if (dx < 1e-4) h = 1.7320508075688773 * diag + 1.4142135623730950 * std::min(dy, dz) + abs(dy - dz);
        if (dy < 1e-4) h = 1.7320508075688773 * diag + 1.4142135623730950 * std::min(dx, dz) + abs(dx - dz);
        if (dz < 1e-4) h = 1.7320508075688773 * diag + 1.4142135623730950 * std::min(dx, dy) + abs(dx - dy);

        h += 0.5 * (x1 - x2).norm();

        return h;
    }


    void Astar::posToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) 
    {
        idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
    }
}
