#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigen>

#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>


namespace AGEL
{
    class SDFMap;
    class Astar;

    struct pair_hash 
    {
        template <typename T1, typename T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const 
        {
            std::size_t seed = 0;
            seed ^= std::hash<T1>()(p.first) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<T2>()(p.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    struct FIS
    {
        int id;

        Eigen::Vector3d average;
        
        Eigen::Vector2d normal;
        Eigen::Vector2d box_min, box_max;

        std::vector<Eigen::Vector2d> cells;

        double cost;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };


    class Frontier
    {
    private:
        std::shared_ptr<SDFMap> map_;
        std::shared_ptr<Astar>  astar_;

        std::vector<bool> is_in_frontier_;

        int     cluster_min_;
        double  cluster_size_xy_;
        double  w_distance_, w_rotation_;
        double  search_short_time_, search_long_time_, search_distance_; 
        double  map_resolution_, box_z_length_, box_z_center_;

        std::mutex frontiers_mutex_;

        Eigen::Vector2d next_best_normal_;
        Eigen::Vector3d next_best_point_;

        bool expandFrontier(const Eigen::Vector2i &start_idx, FIS &fis);

        void computeFrontierAvg(FIS &fis);

        void computeFrontierNormal(FIS &fis);
        
        void splitLargeFrontiers(std::vector<FIS> &fises); 

        bool splitHorizontally(const FIS &frontier, std::vector<FIS> &splits);

        // void downSample(const std::vector<Eigen::Vector3d> &cells_in, std::vector<Eigen::Vector3d> &cells_out);

        double computerCost(Eigen::Vector3d &start_pt, double start_yaw, Eigen::Vector3d &end_pt, double &time_limit);

        bool haveOverlap(const Eigen::Vector2d &min1, const Eigen::Vector2d &max1, 
                         const Eigen::Vector2d &min2, const Eigen::Vector2d &max2);
        
        bool isFrontierChanged(const FIS &ft);

        int idx2Adress(const Eigen::Vector2i &idx);

        bool isFree(const Eigen::Vector2i &idx);

        bool isUnKnow(const Eigen::Vector2i &idx);

        void fourNeighbors(const Eigen::Vector2i &idx, std::vector<Eigen::Vector2i> &neighbors);

        void allNeighbors(const Eigen::Vector2i &idx, std::vector<Eigen::Vector2i> &neighbors);

        bool neighborsHaveUnknown(const Eigen::Vector2i &idx);

        double calcDiffYaw(double &a, double &b);

        void findFrontiers();

        void removeOutDatedFrontiers();

    public:
        std::vector<FIS> frontiers_;

        Frontier(ros::NodeHandle &nh, std::shared_ptr<SDFMap> map);
        ~Frontier();

        void updateFrontiers();

        void updateCost(Eigen::Vector3d &start_pt, double start_yaw);

        void getBestNextFrontierInfo(Eigen::Vector3d &point, Eigen::Vector2d &normal);

        int getFrontiersNumber();


        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}