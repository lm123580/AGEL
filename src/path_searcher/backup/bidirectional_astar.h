#ifndef _ASTAR_H_
#define _ASTAR_H_

#include <ros/ros.h>

#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
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

        enum { REACH_END = 1, NO_PATH = 2, CHANGE_END = 3 };

        void reset();
        int search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);
        int globalSearch(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);
        int reSearch(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);
        void setResolution(const double &res);
        void getGlobalPath(std::vector<Eigen::Vector3d> &path);
        void getLocalPath(std::vector<Eigen::Vector3d> &path);
        double pathLength(std::vector<Eigen::Vector3d> &path);

    private:
        // 参数
        int allocate_num_{};
        double tie_breaker_{};
        double resolution_{}, inv_resolution_{};
        Eigen::Vector3d map_size_3d_, origin_;

        // 主要数据
        std::vector<NodePtr> path_node_pool_;
        int use_node_num_, iter_num_;

        std::vector<Eigen::Vector3d> path_inflate_points_;
        
        // 双向A*
        std::shared_ptr<SDFMap> sdf_map_;
        Eigen::Vector3d start_pt_, end_pt_;
        std::vector<Eigen::Vector3d> search_path_nodes_, global_path_nodes_, local_path_nodes_;
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set_start_, open_set_end_;
        std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> open_set_map_start_, open_set_map_end_;
        std::unordered_map<Eigen::Vector3i, int, matrix_hash<Eigen::Vector3i>> close_set_map_start_, close_set_map_end_;

        // 双向A*实现函数
        bool searchOneDirection(
            std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> &open_set,
            std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> &open_set_map,
            std::unordered_map<Eigen::Vector3i, int, matrix_hash<Eigen::Vector3i>> &close_set_map,
            std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> &other_open_set_map,
            bool forward);

        void backtrack(NodePtr meeting_node_start, NodePtr meeting_node_end);
        void posToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx);
        double getDiagHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);

    public:
        ~Astar();
        Astar(ros::NodeHandle &nh, const std::shared_ptr<SDFMap> &map);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif
