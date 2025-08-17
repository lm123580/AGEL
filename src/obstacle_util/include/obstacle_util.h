#ifndef OBSTACLE_UTIL_H_
#define OBSTACLE_UTIL_H_

#include <ros/ros.h>

#include <deque>
#include <cmath>
#include <limits>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>


namespace AGEL
{
    class SDFMap;
    class CameraUtil;
    class KalmanFilter;
    class DBSCANKdtreeCluster;
    
    class ObstacleUtil
    {
    public:
        enum STATE
        {
            STATIC,
            DYNAMIC
        };

        struct TimeInfo {
            int num{0};
            double total_time{0};
        };

        TimeInfo cluster_time_info_, dada_ass_time_info_,vote_time_info_, beyes_time_info_, classifay_time_info_;
    
    private:
        int print_num_{0};
        Eigen::MatrixXd feature_weighting_matrix_;          // 特征权重矩阵
        double Threshold_diff_;                             // 最高差距阈值
        struct CloudCluster                                 // 点云簇
        {
            ros::Time time;

            std::shared_ptr<CameraUtil> camera_util_ptr;

            STATE priori;

            Eigen::Vector3d quality_point;
            Eigen::Vector3d min_point, max_point;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
            pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ptr;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
        
        struct CSW                                          // 点云簇滑动窗口 Cluster Sliding Window，queue存储不同时刻的同一物体
        {
            STATE state;
            // int alpha, beta;
            double max_x, max_y, max_z;
            std::shared_ptr<KalmanFilter> kf_ptr;
            std::deque<std::shared_ptr<CloudCluster>> cluster_queue;
        };

        std::shared_ptr<SDFMap> map_ptr_;
        std::shared_ptr<DBSCANKdtreeCluster> dbscan_kdtree_ptr_;
        
        void cloud2cluster( std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_vector,
                            std::vector<std::shared_ptr<CloudCluster>> &cluster_vector,
                            std::shared_ptr<CameraUtil> &camera, ros::Time &time);
        void matchObandC(std::vector<std::shared_ptr<CloudCluster>> &cluster_vector, 
                         std::vector<int> &ob2c_indices, std::vector<bool> &cluster_match_flags);
        void getMapPost(std::shared_ptr<CloudCluster> &cluser, STATE &priori);
        void getFDPost(std::shared_ptr<CloudCluster> &cluser_1, std::shared_ptr<CloudCluster> &cluser_2, STATE &priori);
        void filterFov( pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud,
                        std::shared_ptr<CameraUtil> &camera);
        
        void updateTimeInfo(TimeInfo &info, ros::Duration t);
        // void updateKF()
    public:
        std::vector<CSW> cswv_;                              // 点云簇滑动窗口数组，存储不同物体的滑动窗口

        void init(std::shared_ptr<SDFMap> &map);

        void update(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::shared_ptr<CameraUtil> &camera, ros::Time &time);

        void getStaticPCL(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_vector);
        void getDynamicOb(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_vector,
                          std::vector<Eigen::Vector3d> &poses,
                          std::vector<Eigen::Vector3d> &max_sizes,
                          std::vector<std::shared_ptr<KalmanFilter>> &kf_vector);

        ObstacleUtil();
        ~ObstacleUtil();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif
