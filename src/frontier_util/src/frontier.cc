#include "sdf_map.h"
#include "astar.h"
#include "frontier.h"


namespace AGEL
{
    Frontier::Frontier(ros::NodeHandle &nh, std::shared_ptr<SDFMap> map)
    {
        double map_z_size, map_ground_height;

        nh.param("frontier/cluster_min",    cluster_min_,     5);
        nh.param("frontier/cluster_size_xy",cluster_size_xy_, 1.5);

        nh.param("frontier/w_distance", w_distance_, 1.0);
        nh.param("frontier/w_rotation", w_rotation_, 10.0);

        nh.param("frontier/search/short_time", search_short_time_, 0.0005);
        nh.param("frontier/search/long_time",  search_long_time_, 0.0100);
        nh.param("frontier/search/distance",   search_distance_, 10.0);


        nh.param("sdf_map/map_size_z", map_z_size, 2.0);
        nh.param("sdf_map/ground_height", map_ground_height, 0.4);

        map_ = map;
        map_resolution_ = map_->getResolution();

        box_z_length_ = map_z_size - map_ground_height;
        box_z_center_ = (map_z_size + map_ground_height) * 0.50;

        astar_.reset(new Astar(nh, map));

        int vex_num = map_->get2DVoxelNum();
        is_in_frontier_.resize(vex_num, false);
    }


    Frontier::~Frontier() = default;


    void Frontier::removeOutDatedFrontiers()
    {
        // 获取边界更新的范围框
        Eigen::Vector3d search_min, search_max;
        map_->getUpdatedBox(search_min, search_max);

        Eigen::Vector2d search_min_2d, search_max_2d;
        search_min_2d = search_min.head<2>();
        search_max_2d = search_max.head<2>();

        // std::cout << "Before delete frontiers size is: " << frontiers_.size() << std::endl;

        // 删除改变的边界
        std::vector<bool> is_frontier_changed;
        for (const FIS &fis: frontiers_) 
        {
            if (haveOverlap(fis.box_min, fis.box_max, search_min_2d, search_max_2d) || isFrontierChanged(fis)) 
            {
                is_frontier_changed.push_back(true);
            } 
            else 
            {
                is_frontier_changed.push_back(false);
            }
        }
        
        std::vector<FIS> not_changed_frontiers;
        for (size_t i = 0; i < frontiers_.size(); ++i) 
        {
            if (is_frontier_changed[i]) 
            {
                Eigen::Vector2i idx;

                for (const auto &cell: frontiers_[i].cells) 
                {
                    map_->posToIndex(cell, idx);
                    is_in_frontier_[idx2Adress(idx)] = false;
                }
            }
            else 
            {
                not_changed_frontiers.emplace_back(std::move(frontiers_[i]));
            }
        }

        frontiers_ = std::move(not_changed_frontiers);
        // std::cout << "After delete frontiers size is: " << frontiers_.size() << std::endl;
    }

    void Frontier::findFrontiers()
    {
        // ros::Time t1 = ros::Time::now();

        // 获取边界更新的范围框
        Eigen::Vector3d search_min, search_max;
        map_->getUpdatedBox(search_min, search_max);
        // std::cout << "search is from " << search_min.transpose() << " to " << search_max.transpose() << std::endl;

        // 扩大搜索范围
        search_min = search_min - Eigen::Vector3d(1.0, 1.0, 0.0);
        search_max = search_max + Eigen::Vector3d(1.0, 1.0, 0.0);
        map_->boundBox(search_min, search_max);

        // 搜索离散化
        Eigen::Vector3i search_min_idx, search_max_idx;
        map_->posToIndex(search_min, search_min_idx);
        map_->posToIndex(search_max, search_max_idx);

        // 获取2D搜索范围
        Eigen::Vector2i search_2d_min_idx, search_2d_max_idx;
        search_2d_min_idx << search_min_idx[0], search_min_idx[1];
        search_2d_max_idx << search_max_idx[0], search_max_idx[1];

        //搜索前沿
        std::vector<FIS> tmp_fises;
        for (int x = search_2d_min_idx[0]; x <= search_2d_max_idx[0]; x++)
        {
            for (int y = search_2d_min_idx[1]; y <= search_2d_max_idx[1]; y++)
            {
                Eigen::Vector2i cur(x, y);

                // 如果当前节点是边界节点
                if (!is_in_frontier_[idx2Adress(cur)] && isFree(cur) && neighborsHaveUnknown(cur))
                {
                    FIS fis;

                    if (expandFrontier(cur, fis))
                    {
                        tmp_fises.push_back(fis);
                    }
                }
            }
        }

        // 分割前沿
        splitLargeFrontiers(tmp_fises);

        // std::cout << "Find frontiers time is: " << (ros::Time::now() - t1).toSec() * 1000 << "ms" << std::endl;
        // std::cout << "Find frontiers size is: " << tmp_fises.size() << std::endl;
        int idx = 0;
        for (auto &tmp_fis: tmp_fises)
        {
            if ((int)tmp_fis.cells.size() > cluster_min_)
            {
                tmp_fis.id = idx++;
                computeFrontierNormal(tmp_fis);
                frontiers_.push_back(std::move(tmp_fis));                
            }
        }

        // int idx = 0;
        // for (auto &fis: tmp_fises)
        // {
        //     fis.id = idx++;
        // }
    }


    void Frontier::updateFrontiers()
    {
        frontiers_mutex_.lock();
        removeOutDatedFrontiers();
        findFrontiers();
        frontiers_mutex_.unlock();
    }


    void Frontier::splitLargeFrontiers(std::vector<FIS> &fises)
    {
        std::vector<FIS> split_frontiers;

        for (const auto &fis: fises) 
        {
            std::vector<FIS> splits;

            if (splitHorizontally(fis, splits))
            {
                split_frontiers.insert(split_frontiers.end(), splits.begin(), splits.end());
            }
            else
            {
                split_frontiers.emplace_back(std::move(fis));
            }
        }

        fises = std::move(split_frontiers);
    }


    bool Frontier::splitHorizontally(const FIS &fis, std::vector<FIS> &splits)
    {
        // 根据主成分分析分割边界
        bool need_split = false;
        int cells_size = fis.cells.size();
        double cluster_size_xxyy = cluster_size_xy_ * cluster_size_xy_;
        
        // 判断是否需要分割
        Eigen::Vector2d mean = fis.average.head<2>();
        for (const auto &cell: fis.cells) 
        {
            if ((cell - mean).squaredNorm() > cluster_size_xxyy)
            {
                need_split = true;
                break;
            }
        }

        if (!need_split) return false;
        
        Eigen::Matrix2d cov;
        Eigen::Vector2d diff;
        cov.setZero();
        for (const auto &cell: fis.cells) 
        {
            diff = cell - mean;
            cov += diff * diff.transpose();
        }
        cov /= double(cells_size);

        // 查找与最大特征向量相对应的特征向量
        Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
        auto values = es.eigenvalues().real();
        auto vectors = es.eigenvectors().real();
        Eigen::Index max_idx;
        double max_eigenvalue = -1000000.0;
        for (Eigen::Index i = 0; i < values.rows(); ++i) 
        {
            if (values[i] > max_eigenvalue) 
            {
                max_idx = i;
                max_eigenvalue = values[i];
            }
        }
        Eigen::Vector2d dominate_eigen_vector = vectors.col(max_idx);

        FIS fis_1, fis_2;
        for (auto &cell: fis.cells) 
        {
            if ((cell - mean).dot(dominate_eigen_vector) >= 0) 
            {
                fis_1.cells.push_back(cell);
            } 
            else 
            {
                fis_2.cells.push_back(cell);
            }
        }

        computeFrontierAvg(fis_1);
        computeFrontierAvg(fis_2);

        // 如果新的边界仍然过大，则继续分割
        std::vector<FIS> splits_1;
        if (splitHorizontally(fis_1, splits_1)) 
        {
            splits.insert(splits.end(), splits_1.begin(), splits_1.end());
        } 
        else
        {
            splits.push_back(fis_1);
        }

        std::vector<FIS> splits_2;
        if (splitHorizontally(fis_2, splits_2))
        {
            splits.insert(splits.end(), splits_2.begin(), splits_2.end());
        }
        else
        {
            splits.push_back(fis_2);
        }

        return true;
    }


    bool Frontier::expandFrontier(const Eigen::Vector2i &start_idx, FIS &fis)
    {
        std::queue<Eigen::Vector2i> cell_queue;
        std::vector<Eigen::Vector2d> expanded;
        std::vector<Eigen::Vector2i> all_neighbors;
        
        cell_queue.push(start_idx);
        is_in_frontier_[idx2Adress(start_idx)] = true;

        Eigen::Vector2d pos;
        map_->indexToPos(start_idx, pos);
        expanded.push_back(pos);

        // 使用区域增长搜索边界聚类
        while (!cell_queue.empty()) 
        {
            Eigen::Vector2i cur_cell = cell_queue.front();
            cell_queue.pop();

            allNeighbors(cur_cell, all_neighbors);
            for (const auto &next_cell: all_neighbors) 
            {
                // 新的边界点是Free且周围有未知状态的邻居
                int next_cell_adr = idx2Adress(next_cell);
                if (is_in_frontier_[next_cell_adr] || //!map_->isInBox(next_cell) || 
                  !(isFree(next_cell) && neighborsHaveUnknown(next_cell)))
                {
                    continue;
                }

                cell_queue.push(next_cell);

                map_->indexToPos(next_cell, pos);

                expanded.push_back(pos);
                
                is_in_frontier_[next_cell_adr] = true;
            }
        }

        if ((int)expanded.size() > cluster_min_) 
        {
            fis.cells = std::move(expanded);
            computeFrontierAvg(fis);
            
            return true;
        }

        return false;
    }


    void Frontier::computeFrontierAvg(FIS &fis)
    {
        fis.average.setZero();
        Eigen::Vector2d tmp_average;

        if (fis.cells.size() == 0)
        {
            return ;
        }

        fis.box_max = fis.cells.front();
        fis.box_min = fis.cells.front();
        for (const auto &cell: fis.cells) 
        {
            tmp_average += cell;
            for (Eigen::Index i = 0; i < 2; ++i) 
            {
                fis.box_min[i] = std::min(fis.box_min[i], cell[i]);
                fis.box_max[i] = std::max(fis.box_max[i], cell[i]);
            }
        }

        tmp_average /= double(fis.cells.size());

        fis.average = Eigen::Vector3d(tmp_average[0], tmp_average[1], box_z_center_);
    }


    void Frontier::computeFrontierNormal(FIS &fis)
    {
        // 根据主成分分析先得到法向量
        int cells_size;
        Eigen::Vector2d mean;

        Eigen::Matrix2d cov;
        Eigen::Vector2d diff;
        cov.setZero();

        mean = fis.average.head<2>();
        cells_size = fis.cells.size();
        for (const auto &cell: fis.cells) 
        {
            diff = cell - mean;
            cov += diff * diff.transpose();
        }
        cov /= double(cells_size);

        // 查找与最大特征向量相对应的特征向量
        Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
        auto values = es.eigenvalues().real();
        auto vectors = es.eigenvectors().real();
        Eigen::Index max_idx;
        double max_eigenvalue = -1000000.0;
        for (Eigen::Index i = 0; i < values.rows(); ++i) 
        {
            if (values[i] > max_eigenvalue) 
            {
                max_idx = i;
                max_eigenvalue = values[i];
            }
        }
        Eigen::Vector2d dominate_eigen_vector = vectors.col(max_idx);
        dominate_eigen_vector.normalize();


        // 两个正交法向量
        Eigen::Vector2i tmp_idx;
        Eigen::Vector2d tmp_pos, tmp_normal;
        Eigen::Vector2d normal_vector_1(-dominate_eigen_vector[1], dominate_eigen_vector[0]);
        Eigen::Vector2d normal_vector_2(dominate_eigen_vector[1], -dominate_eigen_vector[0]);

        tmp_normal = normal_vector_1;
        double delta_length = map_resolution_ * 0.5;
        for (double length = delta_length; length <= 2.0; length += delta_length)
        {
            // 第一个方向
            tmp_pos = mean + normal_vector_1 * length;
            map_->posToIndex(tmp_pos, tmp_idx);
            if (isUnKnow(tmp_idx))
            {
                tmp_normal = normal_vector_1;
                break;
            }

            // 第二个方向
            tmp_pos = mean + normal_vector_2 * length;
            map_->posToIndex(tmp_pos, tmp_idx);
            if (isUnKnow(tmp_idx))
            {
                tmp_normal = normal_vector_2;
                break;
            }
        }

        fis.normal = tmp_normal;
    }


    void Frontier::updateCost(Eigen::Vector3d &start_pt, double start_yaw)
    {
        double min_cost = std::numeric_limits<double>::max();

        double inf = std::numeric_limits<double>::infinity();
        Eigen::Vector2d next_best_normal;
        Eigen::Vector3d next_best_point(inf, inf, inf);

        double sq_distance = search_distance_ * search_distance_;
        
        frontiers_mutex_.lock();
        // 1. 先初步搜索周围的前沿，时间短  附近的前沿会更新
        for (FIS &fis: frontiers_) 
        {
            if (!map_->getInflateOccupancy(fis.average) && (fis.average - start_pt).squaredNorm() < sq_distance)
            {
                fis.cost = computerCost(start_pt, start_yaw, fis.average, search_short_time_);
                
                if (fis.cost < min_cost)
                {
                    min_cost = fis.cost;
                    next_best_point = fis.average;
                    next_best_normal = fis.normal;
                }
            }
        }
        
        if (min_cost != std::numeric_limits<double>::max())
        {
            next_best_normal_ = next_best_normal;
            next_best_point_ = next_best_point;
            frontiers_mutex_.unlock();

            return;
        }

        // 2. 如果周围不存在可更新的前沿，那么就可以放心搜索前沿
        for (FIS &fis: frontiers_) 
        {
            if (map_->getInflateOccupancy(fis.average)) continue;
            
            fis.cost = computerCost(start_pt, start_yaw, fis.average, search_long_time_);
            
            if (fis.cost < min_cost)
            {
                min_cost = fis.cost;
                next_best_point = fis.average;
                next_best_normal = fis.normal;
            }                
        }
        next_best_normal_ = next_best_normal;
        next_best_point_ = next_best_point;
        frontiers_mutex_.unlock();
    }


    void Frontier::getBestNextFrontierInfo(Eigen::Vector3d &point, Eigen::Vector2d &normal)
    {
        point = next_best_point_;
        normal = next_best_normal_;
    }

    int Frontier::getFrontiersNumber()
    {
        int reachable_num = 0;
        for (auto &fis: frontiers_)
        {
            if (fis.cost != std::numeric_limits<double>::max())
            {
                reachable_num++;
            }
        }

        if (reachable_num == 0)
        {
            frontiers_.clear();
        }
        
        return reachable_num;
    }


    double Frontier::computerCost(Eigen::Vector3d &start_pt, double start_yaw, Eigen::Vector3d &end_pt, double &time_limit)
    {
        double cost = 0.0;
        std::vector<Eigen::Vector3d> path;

        if (astar_->globalPlan(start_pt, end_pt, time_limit))
        {
            astar_->getPath(path);
            std::vector<Eigen::Vector3d> dir_vector;

            int path_size = path.size();
            for (int i = 0; i < path_size - 1; i++)
            {
                dir_vector.emplace_back(path[i+1] - path[i]);
            }

            // 计算距离cost
            for (auto &vector: dir_vector)
            {
                cost = cost + w_distance_ * vector.norm();
            }

            // 计算角度cost
            double weight = w_rotation_;
            double last_yaw, now_yaw;
            last_yaw = start_yaw;
            for (auto &vector: dir_vector)
            {
                now_yaw = std::atan2(vector[1], vector[0]);
                cost += weight * calcDiffYaw(last_yaw, now_yaw);
                weight *= 0.50;
                if (weight < 1.0)
                {
                    weight = 1.0;
                }
                last_yaw = now_yaw;
            }
        }
        else
        {
            cost = std::numeric_limits<double>::max();
        }

        return cost;
    }


    double Frontier::calcDiffYaw(double &a, double &b)
    {
        if (a - b > M_PI)       return (2 * M_PI - (a - b));
        else if (b - a > M_PI)  return (2 * M_PI - (b - a));
        else if (a > b)         return (a - b);
        else                    return (b - a);

        return (b - a);
    }


    bool Frontier::haveOverlap( const Eigen::Vector2d &min1, const Eigen::Vector2d &max1, 
                                const Eigen::Vector2d &min2, const Eigen::Vector2d &max2)
    {
        // 检查两个包围框是否相交
        double box_min, box_max;
        for (Eigen::Index i = 0; i < 2; ++i) 
        {
            box_min = std::max(min1[i], min2[i]);
            box_max = std::min(max1[i], max2[i]);
            
            if (box_min > box_max + 1e-3) 
            {
                return false;
            }
        }

        return true;
    }


    bool Frontier::isFrontierChanged(const FIS &fis)
    {
        Eigen::Vector2i idx;
        
        for (const auto &cell: fis.cells)
        {
            map_->posToIndex(cell, idx);

            if (!(isFree(idx) && neighborsHaveUnknown(idx))) 
            {
                return true;
            }
        }

        return false;
    }


    inline int Frontier::idx2Adress(const Eigen::Vector2i &idx)
    {
        return map_->to2DAddress(idx[0], idx[1]);
    }

    
    inline bool Frontier::isFree(const Eigen::Vector2i &idx)
    {
        return (map_->get2DState(idx) == SDFMap::FREE);
    }


    inline bool Frontier::isUnKnow(const Eigen::Vector2i &idx)
    {
        return (map_->get2DState(idx) == SDFMap::UNKNOWN);
    }


    inline void Frontier::fourNeighbors(const Eigen::Vector2i &idx, std::vector<Eigen::Vector2i> &neighbors) 
    {
        neighbors.resize(4);

        neighbors[0] = idx - Eigen::Vector2i(1, 0);
        neighbors[1] = idx + Eigen::Vector2i(1, 0);
        neighbors[2] = idx - Eigen::Vector2i(0, 1);
        neighbors[3] = idx + Eigen::Vector2i(0, 1);
    }


    inline void Frontier::allNeighbors(const Eigen::Vector2i &idx, std::vector<Eigen::Vector2i> &neighbors) 
    {
        neighbors.resize(8);

        int count = 0;
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                if (x == 0 && y == 0)
                {
                    continue;
                }

                neighbors[count++] = idx + Eigen::Vector2i(x, y);
            }
        }
    }

    
    inline bool Frontier::neighborsHaveUnknown(const Eigen::Vector2i &idx)
    {
        std::vector<Eigen::Vector2i> four_neighbors;

        fourNeighbors(idx, four_neighbors);
        
        for (const auto &neighbor : four_neighbors)
        {
            if (isUnKnow(neighbor))
            {
                return true;
            }
        }

        return false;
    }
}
