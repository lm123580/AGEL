#include "sdf_map.h"
#include "dstar.h"

namespace AGEL
{
    Dstar::Dstar(ros::NodeHandle &nh, const std::shared_ptr<SDFMap> &map)
    {
        nh.param("astar/resolution_astar", resolution_, 0.40);
        // nh.param("astar/lambda_heu", lambda_heu_, 1.5);
        nh.param("astar/max_search_time", max_search_time_, 0.05);
        nh.param("astar/allocate_num", allocate_num_, 1000000);
        nh.param("astar/range", range_, 2.0);

        map_ = map;
        map->getRegion(origin_, map_size_3d_);

        inv_resolution_ = 1.0 / resolution_;

        map_resolution_ = map->getResolution();
        map_step_ = std::ceil(resolution_ / map_resolution_);

        k_m_ = 0.0;
        use_node_num_ = 0;
    }

    
    Dstar::~Dstar() = default;


    bool Dstar::globalPlan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt)
    {
        reset();
        path_nodes_.clear();
        
        Key tmp_key;

        // 设置终点
        NodePtr end_node = std::make_shared<Node>();
        end_node_ = end_node;
        end_node->position = end_pt;
        posToIndex(end_pt, end_node->index);
        end_node->rhs = 0.0;
        nodes_map_[end_node->index] = end_node;
        tmp_key.k1 = heuristic(start_pt, end_pt);
        tmp_key.k2 = 0.0;
        insertNode(end_node, tmp_key);
        
        // 设置起点
        NodePtr start_node = std::make_shared<Node>();
        start_node_ = start_node;
        start_node->position = start_pt;
        posToIndex(start_pt, start_node->index);
        nodes_map_[start_node->index] = start_node;

        last_node_ = start_node;

        // 寻找最短路径
        if (computeShortestPath())
        {
            NodePtr cur_node = start_node_;
            while (cur_node->index != end_node_->index)
            {
                std::vector<NodePtr> succs;
                path_nodes_.push_back(cur_node->position);

                getSucc(cur_node->position, succs);
                if (succs.empty())
                {
                    std::cout << "Wrong, path was occupied!!!" << std::endl;
                    return false;
                }

                double min_cost = INFINITY;
                double tmp_cost;
                NodePtr next_node;

                for (auto &succ: succs)
                {
                    tmp_cost = getG(succ);
                    if (tmp_cost < min_cost)
                    {
                        min_cost = tmp_cost;
                        next_node = succ;
                    }
                }

                cur_node = next_node;
            }
            path_nodes_.push_back(end_node_->position);

            return true;
        }

        return false;
    }


    bool Dstar::replan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt)
    {
        int path_size = path_nodes_.size();

        bool occ_flag = false;

        NodePtr tmp_node;
        Eigen::Vector3i node_idx;
        std::vector<NodePtr> succs;
        for (int i = 1; i < path_size; i++)
        {
            // std::cout << "ss" << std::endl;
            if (isOccupied(path_nodes_[i-1], path_nodes_[i]))
            {
                posToIndex(path_nodes_[i], node_idx);

                makeNewNode(node_idx, path_nodes_[i]);
                
                tmp_node = nodes_map_[node_idx];
                
                setRHS(tmp_node ,INFINITY);
                
                // getSucc(path_nodes_[i-1], succs);

                // double min_rhs = INFINITY, tmp_rhs;
                // for (auto &succ: succs)
                // {
                //     tmp_rhs = heuristic(path_nodes_[i-1], succ->position) + getG(succ);
                //     if (tmp_rhs < min_rhs)
                //     {
                //         min_rhs = tmp_rhs;
                //     }
                // }
                // setRHS(tmp_node, min_rhs);

                // std::cout << "min rhs is:" << min_rhs << std::endl;

                updateVertex(tmp_node);

                occ_flag = true;
            }
        }

        if (occ_flag)
        {
            path_nodes_.clear();
            open_queue_.clear();
            open_list_.clear();

            Key key;
            key = calculateKey(end_node_);
            insertNode(end_node_, key);
            
            k_m_ = k_m_ + heuristic(last_node_->position, start_pt);

            posToIndex(start_pt, node_idx);

            makeNewNode(node_idx, start_pt);

            start_node_ = nodes_map_[node_idx];

            last_node_ = start_node_;

            if (computeShortestPath())
            {
                std::cout << "sdfsdfdsfsdf" << std::endl;
                NodePtr cur_node = start_node_;
                while (cur_node->index != end_node_->index)
                {
                    std::vector<NodePtr> succs;
                    path_nodes_.push_back(cur_node->position);

                    getSucc(cur_node->position, succs);
                    if (succs.empty())
                    {
                        std::cout << "Wrong, path was occupied!!!" << std::endl;
                        return false;
                    }

                    double min_cost = INFINITY;
                    double tmp_cost;
                    NodePtr next_node;

                    for (auto &succ: succs)
                    {
                        tmp_cost = getG(succ);
                        if (tmp_cost < min_cost)
                        {
                            min_cost = tmp_cost;
                            next_node = succ;
                        }
                    }

                    cur_node = next_node;
                }
                path_nodes_.push_back(end_node_->position);
            }
           else
           {
                return false;
           }
        }

        return true;
    }


    void Dstar::getChangeNodes(Eigen::Vector3d &pos, std::vector<Eigen::Vector3i> nodes)
    {
        // 得到pos周围的占据节点坐标，注意map_resolution和D*搜索时的resolution
        Eigen::Vector3i pos_idx;
        posToIndex(pos, pos_idx);

        std::vector<Eigen::Vector3i> tmp_nodes;
        std::set<Eigen::Vector3i> tmp_nodes_set;

        // 得到点云
        for (int x = -20; x <= 20; x++)
        {
            for (int y = -20; y <= 20; y++)
            {
                for (int z = 0; z <= 16; z++)
                {
                    Eigen::Vector3i tmp_idx = pos_idx + Eigen::Vector3i(x, y, z);
                    tmp_idx[2] = z;

                    if (map_->getInflateOccupancy(tmp_idx))
                    {
                        tmp_nodes.push_back(tmp_idx);
                    }
                }
            }
        }

        // // 将map_resolution下nodes转化为D*搜索时的resolution下的nodes
        // Eigen::vector3d tmp_pos;
        // Eigen::vector3i tmp_idx;
        // for (auto &node: tmp_nodes)
        // {
        //     tmp_pos = node.cast<double>() * 0.1;
        //     posToIndex(tmp_pos, tmp_idx);
        //     tmp_nodes_set.insert(tmp_idx);
        // }
        

        // for (auto &node: tmp_nodes_set)
        // {
        //     if (nodes_map_.find(node) != nodes_map_.end())
        //     {
        //     }
        // }
        
    }

    bool Dstar::replan()
    {
        computeShortestPath();

        if (isinf(getG(start_node_)))
        {
            std::cout << "NO PATH TO GOAL!!!" << std::endl;

            return false;
        }

        path_nodes_.clear();


        NodePtr cur_node = start_node_;
        while (cur_node->index != end_node_->index)
        {
            std::vector<NodePtr> succs;
            path_nodes_.push_back(cur_node->position);

            getSucc(cur_node->position, succs);
            if (succs.empty())
            {
                std::cout << "Wrong, path was occupied!!!" << std::endl;
                return false;
            }

            double min_cost = INFINITY;
            double tmp_cost;
            NodePtr next_node;

            for (auto &succ: succs)
            {
                tmp_cost = getG(succ);
                if (tmp_cost < min_cost)
                {
                    min_cost = tmp_cost;
                    next_node = succ;
                }
            }

            cur_node = next_node;
        }
        path_nodes_.push_back(end_node_->position);

        return true;
    }


    void Dstar::updateStart(Eigen::Vector3d &new_start)
    {
        k_m_ += heuristic(last_node_->position, new_start);

        Eigen::Vector3i new_start_idx;
        posToIndex(new_start, new_start_idx);
        makeNewNode(new_start_idx, new_start);

        start_node_ = nodes_map_[new_start_idx];
        last_node_ = start_node_;

        // while (!open_queue_.empty())
        // {
        //     open_queue_.pop();
        // }
        // open_list_.clear();
    }


    bool Dstar::computeShortestPath()
    {
        while ( (!open_queue_.empty()) &&
                ((open_queue_.top()->key < calculateKey(start_node_)) ||
                (getRHS(start_node_) != getG(start_node_))))
        {
            NodePtr cur_node = open_queue_.top();

            std::cout << "cur node position is: " << cur_node->position << std::endl;

            Key k_old = cur_node->key;
            Key k_new = calculateKey(cur_node);

            if(k_old < k_new)
            {
                updateNode(cur_node, k_new);
            }
            else
            {
                double cur_g = getG(cur_node);
                double cur_rhs = getRHS(cur_node);
                std::vector<NodePtr> preds;

                if (cur_g == cur_rhs)
                {
                    removeTop();
                }
                else if (cur_g > cur_rhs)
                {
                    // removeTop();

                    setG(cur_node, cur_rhs);
                    cur_g = getG(cur_node);

                    removeNode(cur_node);

                    getPred(cur_node->position, preds);

                    for (auto &pred: preds)
                    {
                        if (*pred != *end_node_)
                        {
                            double pred_rhs = std::fmin(getRHS(pred), heuristic(pred->position, cur_node->position) + cur_g);
                            setRHS(pred, pred_rhs);
                        }

                        updateVertex(pred);
                    }
                }
                else
                {
                    double g_old = cur_g;
                    setG(cur_node, INFINITY);

                    getPred(cur_node->position, preds);
                    preds.push_back(cur_node);
                    
                    for (auto &pred: preds)
                    {
                        std::vector<NodePtr> succs;

                        double pred_rhs = getRHS(pred);
                        if (pred_rhs == heuristic(pred->position, cur_node->position) + g_old && pred != end_node_)
                        {
                            double succ_rhs;
                            double min_succ_rhs = INFINITY;
                            
                            getSucc(pred->position, succs);
                            
                            for (auto &succ: succs)
                            {
                                succ_rhs = heuristic(pred->position, succ->position) + getG(succ);
                                if (min_succ_rhs < succ_rhs)
                                {
                                    min_succ_rhs = succ_rhs;
                                }
                            }
                        }

                        updateVertex(pred);
                    }
                }
            }
        }

        if (getRHS(start_node_) != getG(start_node_))
        {
            return false;
        }

        return true;
    }

    void Dstar::makeNewNode(const Eigen::Vector3i &node_index, const Eigen::Vector3d &node_position)
    {
        if (nodes_map_.find(node_index) != nodes_map_.end())   return;

        NodePtr node = std::make_shared<Node>();
        node->index = node_index;
        node->position = node_position;

        nodes_map_[node_index] = node;
    }


    void Dstar::getPred(const Eigen::Vector3d &cur_pos, std::vector<NodePtr> &preds)
    {
        Eigen::Vector3i pred_idx, cur_idx;
        Eigen::Vector3d pred_pos, step;

        preds.clear();

        for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
        {
            for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
            {
                for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_)
                {
                    step << dx, dy, dz;
                    
                    if (step.squaredNorm() < 1e-6) continue;
                    
                    pred_pos = cur_pos + step;

                    if (isOccupied(cur_pos, pred_pos)) continue;

                    // if (isOccupied(cur_pos, pred_pos)) // continue;
                    // {
                    //     posToIndex(pred_pos, pred_idx);
                        
                    //     makeNewNode(pred_idx, pred_pos);
                        
                    //     setRHS(nodes_map_[pred_idx], INFINITY);

                    //     continue;
                    // }
                    // if (map_->getInflateOccupancy(pred_pos) == true) continue;

                    // step.normalize();
                    // bool safe = true;
                    // Eigen::Vector3d tmp_pos;
                    // for (double i = 0; i <= 0.4 + 1e-3; i += 0.05)
                    // {
                    //     tmp_pos = cur_pos + step * i;
                    //     if (map_->getInflateOccupancy(tmp_pos) == true)
                    //     {
                    //         safe = false;
                    //     }
                    // }

                    // if (!safe)  continue;

                    
                    posToIndex(pred_pos, pred_idx);
                    
                    makeNewNode(pred_idx, pred_pos);

                    preds.push_back(nodes_map_[pred_idx]);
                }
            }
        }
    }


    void Dstar::getSucc(const Eigen::Vector3d &cur_pos, std::vector<NodePtr> &succs)
    {
        succs.clear();

        if (isOccupied(cur_pos)) return;
        // if (map_->getInflateOccupancy(cur_pos) == true) return;

        Eigen::Vector3i succ_idx;
        Eigen::Vector3d succ_pos, step;

        for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
        {
            for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
            {
                for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_)
                {
                    step << dx, dy, dz;
                    if (step.squaredNorm() < 1e-6) continue;

                    succ_pos = cur_pos + step;

                    if (isOccupied(cur_pos, succ_pos)) continue;
                    // if (isOccupied(cur_pos, succ_pos)) // continue;
                    // {
                    //     posToIndex(succ_pos, succ_idx);
                        
                    //     makeNewNode(succ_idx, succ_pos);
                        
                    //     setRHS(nodes_map_[succ_idx], INFINITY);
                    // }

                    posToIndex(succ_pos, succ_idx);
                    
                    makeNewNode(succ_idx, succ_pos);

                    succs.push_back(nodes_map_[succ_idx]);
                }
            }
        }
    }


    void Dstar::updateVertex(NodePtr &node) 
    {
        bool in_open_ = (open_list_.find(node->index) != open_list_.end());
        
        double node_g = getG(node);
        double node_rhs = getRHS(node);
        
        if (node_g != node_rhs && in_open_)
        {
            // node->key = calculateKey(node);
            Key key = calculateKey(node);

            updateNode(node, key);
        }
        else if (node_g != node_rhs && !in_open_)
        {
            // node->key = calculateKey(node);
            // insert(node);
            Key key = calculateKey(node);
    
            insertNode(node, key);
        }
        else if (node_g == node_rhs && in_open_)
        {
            removeNode(node);
        }
    }

    bool Dstar::isOccupied(const Eigen::Vector3d &pos)
    {
        return map_->getInflateOccupancy(pos);
    }

    bool Dstar::isOccupied(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2)
    {
        Eigen::Vector3d dir = pos2 - pos1, tmp_pos = pos1;
        double dir_length = dir.norm();
        
        int step = std::ceil(dir_length / map_resolution_) + 1;

        dir = dir.normalized() * map_resolution_;
        for (int i = 0; i <= step; i++)
        {
            if (map_->getInflateOccupancy(tmp_pos))
            {
                return true;
            }

            tmp_pos += dir;
        }

        return false;
    }

    void Dstar::removeNode(NodePtr &node)
    {
        open_queue_.erase(node);
        open_list_.erase(node->index);
    }


    void Dstar::updateNode(NodePtr &node, Key &key)
    {
        removeNode(node);
        
        NodePtr new_node = std::make_shared<Node>(*node);

        nodes_map_[new_node->index] = new_node;

        insertNode(new_node, key);
    }


    void Dstar::insertNode(NodePtr &node, Key &key) 
    {
        node->key = key;
        open_queue_.push(node);
        open_list_.insert(node->index);
    }


    void Dstar::removeTop()
    {
        open_list_.erase((open_queue_.top()->index));
        open_queue_.pop();
    }
    
    
    bool Dstar::isValid(NodePtr &node) 
    {
        if (open_list_.find(node->index) == open_list_.end()) return false;
        
        return true;
    }


    void Dstar::setG(NodePtr &node, double g)
    {
        // makeNewNode(node->index, node->position);
        nodes_map_[node->index]->g = g;
    }


    void Dstar::setRHS(NodePtr &node, double rhs)
    {
        // makeNewNode(node->index, node->position);
        nodes_map_[node->index]->rhs = rhs;
    }


    double Dstar::getG(const NodePtr &node) 
    {
        // if (nodes_map_.find(node->index) == nodes_map_.end())   {std::cout << "get G empty" << std::endl; return heuristic(node->position, end_node_->position);}
        return nodes_map_[node->index]->g;
    }


    double Dstar::getRHS(const NodePtr &node)
    {
        // if (nodes_map_.find(node->index) == nodes_map_.end())   {std::cout << "get RHS empty" << std::endl; return heuristic(node->position, end_node_->position);}
        return nodes_map_[node->index]->rhs;
    }

    Key Dstar::calculateKey(NodePtr &node)
    {
        Key key;
        double val = std::fmin(node->g, node->rhs);

        key.k1 = val + heuristic(node->position, start_node_->position) + k_m_;
        key.k2 = val;

        return key;
    }


    double Dstar::heuristic(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2) 
    {
        return getDiagHeu(x1, x2);
    }


    double Dstar::getDiagHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2) 
    {
        double h  = 0;
        double dx = fabs(x1(0) - x2(0));
        double dy = fabs(x1(1) - x2(1));
        double dz = fabs(x1(2) - x2(2));
        double diag = std::min(std::min(dx, dy), dz);

        dx -= diag;
        dy -= diag;
        dz -= diag;

        // sqrt(3.0) = 1.7320508075688773
        // sqrt(2.0) = 1.4142135623730950
        // if (dx < 1e-4) h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dy, dz) + 1.0 * abs(dy - dz);
        // if (dy < 1e-4) h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dz) + 1.0 * abs(dx - dz);
        // if (dz < 1e-4) h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * std::min(dx, dy) + 1.0 * abs(dx - dy);

        if (dx < 1e-4) h = 1.7320508075688773 * diag + 1.4142135623730950 * std::min(dy, dz) + abs(dy - dz);
        if (dy < 1e-4) h = 1.7320508075688773 * diag + 1.4142135623730950 * std::min(dx, dz) + abs(dx - dz);
        if (dz < 1e-4) h = 1.7320508075688773 * diag + 1.4142135623730950 * std::min(dx, dy) + abs(dx - dy);

        return h;
    }


    void Dstar::posToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) 
    {
        idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
    }
    
    void Dstar::reset()
    {
        open_queue_.clear();
        open_list_.clear();
        nodes_map_.clear();
        path_nodes_.clear();

        k_m_ = 0.0;
    }

    void Dstar::getPath(std::vector<Eigen::Vector3d> &path)
    {
        path = path_nodes_;
    }
}