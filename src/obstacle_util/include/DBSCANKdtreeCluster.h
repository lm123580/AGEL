#ifndef DBSCAN_KDTREE_CLUSTER_H_
#define DBSCAN_KDTREE_CLUSTER_H_

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>

namespace AGEL
{
    class DBSCANKdtreeCluster
    {
    private:
        enum STATE
        {
            UN_PROCESSED, PROCESSING, PROCESSED
        };

        float eps_;
        int minPts_, min_pts_per_cluster_, max_pts_per_cluster_;

    public:
        void run(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_vector);
        DBSCANKdtreeCluster();
        ~DBSCANKdtreeCluster()
        {
        }
    };    
}

#endif