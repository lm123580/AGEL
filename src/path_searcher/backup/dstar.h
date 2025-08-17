#ifndef _DSTAR_H_
#define _DSTAR_H_

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

    struct Key
    {
        double k1, k2;

        bool operator > (const Key &key2) const 
        {
            if (k1 - 1e-6 > key2.k1) return true;
            else if (k1 < key2.k1 - 1e-6) return false;

            return k2 > key2.k2;
        }


        bool operator <= (const Key &key2) const 
        {
            if (k1 < key2.k1) return true;
            else if (k1 > key2.k1) return false;

            return k2 < key2.k2 + 1e-6;
        }

        bool operator < (const Key &key2) const 
        {
            if (k1 + 1e-6 < key2.k1) return true;
            else if (k1 - 1e-6 > key2.k1) return false;

            return k2 < key2.k2;
        }
    };

    struct Node
    {
        Eigen::Vector3i index;
        Eigen::Vector3d position;

        Key key;
        double g = INFINITY, rhs = INFINITY;

        bool operator == (const Node &node2) const 
        {
            return (index == node2.index);
        }

        bool operator != (const Node &node2) const 
        {
            return (index != node2.index);
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    typedef std::shared_ptr<Node> NodePtr;

    class NodeComparator 
    {
    public:
        bool operator()(const NodePtr& node1, const NodePtr& node2) 
        {
            return (node1->key > node2->key);
        }
    };

    struct PriorityQueue
    {
        std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> data, remove_data;
        
        void pop()              { data.pop(); }
        bool empty()            { return data.empty(); }
        void push(NodePtr &x)   { data.push(x); }
        void erase(NodePtr &x)  { remove_data.push(x); }

        void clear()            
        { 
            std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> empty_1, empty_2;

            data.swap(empty_1);
            remove_data.swap(empty_2);
        }

        NodePtr top() 
        {
            while(!remove_data.empty() && !data.empty() && data.top() == remove_data.top()) {data.pop(); remove_data.pop();}
            
            return data.empty() ? nullptr: data.top();
        }
    };


    class Dstar
    {
    private:
        // 必要参数
        double lambda_heu_{};
        double max_search_time_{}, range_{};
        double resolution_{}, inv_resolution_{};
        int map_step_{};
        double map_resolution_{};
        Eigen::Vector3d origin_{}, map_size_3d_{};

        int allocate_num_{};

        // 数据
        std::shared_ptr<SDFMap> map_;

        double k_m_{};
        int use_node_num_{};

        NodePtr start_node_, end_node_, last_node_;

        std::vector<Eigen::Vector3d> path_nodes_;

        PriorityQueue open_queue_;
        std::unordered_set<Eigen::Vector3i, matrix_hash<Eigen::Vector3i>> open_list_;
        std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> nodes_map_;

        Key calculateKey(NodePtr &node);
        
        void updateVertex(NodePtr &node);
        
        bool isValid(NodePtr &node);

        void removeNode(NodePtr &node);
        
        void updateNode(NodePtr &node, Key &key);
        
        // void insert(NodePtr &node);

        void insertNode(NodePtr &node, Key &key);

        void removeTop();        
        
        double getDiagHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);

        double heuristic(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);        
        
        void makeNewNode(const Eigen::Vector3i &node_index, const Eigen::Vector3d &node_position);

        bool isOccupied(const Eigen::Vector3d &pos);
        
        bool isOccupied(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2);
        
        void setG(NodePtr &node, double g);

        void setRHS(NodePtr &node, double rhs);

        double getG(const NodePtr &node);

        double getRHS(const NodePtr &node);        
        
        void getPred(const Eigen::Vector3d &cur_pos, std::vector<NodePtr> &preds);

        void getSucc(const Eigen::Vector3d &cur_pos, std::vector<NodePtr> &succs);

        void getChangeNodes(Eigen::Vector3d &pos, std::vector<Eigen::Vector3i> nodes);

        void reset();
        
        void posToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx);

    public:
        Dstar(ros::NodeHandle &nh, const std::shared_ptr<SDFMap> &map);
        
        ~Dstar();
        
        bool globalPlan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);

        bool replan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt);

        
        bool replan();

        bool computeShortestPath();

        void updateStart(Eigen::Vector3d &new_start);
        
        void getPath(std::vector<Eigen::Vector3d> &path);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif