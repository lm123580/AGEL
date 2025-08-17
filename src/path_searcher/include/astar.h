#ifndef _ASTAR_H_
#define _ASTAR_H_

#include <ros/ros.h>

#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <unordered_set>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>
#include <memory>

#include <Eigen/Eigen>

namespace AGEL
{
    class SDFMap;


    template <typename T>
    struct matrix_hash : std::unary_function<T, size_t> 
    {
        std::size_t operator()(T const& matrix) const 
        {
            size_t seed = 0;
            for (size_t i = 0; i < matrix.size(); ++i) 
            {
                auto elem = *(matrix.data() + i);
                seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            
            return seed;
        }
    };


    struct Node
    {
        Eigen::Vector3i index;
        Eigen::Vector3d position;
        double g_score{}, f_score{};

        std::shared_ptr<Node> parent = nullptr;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };


    typedef std::shared_ptr<Node> NodePtr;


    class NodeComparator 
    {
    public:
        bool operator()(const NodePtr& node1, const NodePtr& node2) 
        {
            return node1->f_score > node2->f_score;
        }
    };


    class Astar
    {
    public:
        double lambda_heu_{};
        double max_search_time_{};

        void    reset();
        
        // bool    search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);

        bool    search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt, const double &time_limit);
        
        bool    globalPlan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);

        bool    globalPlan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt, const double &time_limit);
        
        bool    replan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);
        
        void    setResolution(const double &res);
        
        void    getPath(std::vector<Eigen::Vector3d> &path);
        
        double  pathLength();

    private:
        double  margin_{};
        int     allocate_num_{};
        double  tie_breaker_{};
        double  resolution_{}, inv_resolution_{};

        std::shared_ptr<SDFMap> map_;
        int     check_occ_step_{};
        double  map_resolution_{};
        Eigen::Vector3d map_size_3d_, origin_;

        std::vector<double> search_step_length_;
        std::vector<Eigen::Vector3d> search_neighbors_;

        int use_node_num_;
        std::vector<NodePtr> path_node_pool_;
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_queue_;
        std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> open_map_set_;
        std::unordered_set<Eigen::Vector3i, matrix_hash<Eigen::Vector3i>> close_set_;

        std::vector<Eigen::Vector3d> path_nodes_;
        std::vector<Eigen::Vector3d> search_path_nodes_;
        std::vector<Eigen::Vector3d> global_path_nodes_;

        void    backtrack(const NodePtr &end_node, const Eigen::Vector3d &end);
        
        bool    isOccupied(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2);
        
        void    posToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx);
        
        double  getDiagHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);
        
    public:
        ~Astar();
        
        Astar(ros::NodeHandle &nh, const std::shared_ptr<SDFMap> &map);
    
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif
