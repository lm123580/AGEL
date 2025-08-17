#ifndef MAP_MANAGER_H_
#define MAP_MANAGER_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <set>
#include <memory>
#include <random>


namespace AGEL
{
    class KalmanFilter;
    class CameraUtil;
    class SDFMap;
    class ObstacleUtil;
    class Frontier;

    class MapManager
    {
    public:
        struct DynamicObject                                    // 动态物体
        {
            Eigen::Vector3d pos;                                // 位置
            Eigen::Vector3d max_size;                           // AABB
            std::shared_ptr<KalmanFilter> kf_ptr;               // 卡尔曼滤波器
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;      // 点云(待选项)
            
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
        struct TimeInfo {
            int num{0};
            double total_time{0};
        };

        TimeInfo map_dyn_updat_time_info_;

        std::shared_ptr<SDFMap> map_;
        std::vector<DynamicObject> dynamic_objects_;

    private:
        int info_num_{0}, pub_frontier_num_{0};
        ros::Time start_time_;
        std::shared_ptr<CameraUtil> camera_;
        std::shared_ptr<ObstacleUtil> obstacle_;
        visualization_msgs::MarkerArray mk_dynamics_;
        visualization_msgs::MarkerArray mk_frontiers_;

        double map_ground_filter_hight_;

        int debug_{};

        ros::Time debug_time_1_{}, debug_time_2_{};

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> SyncPolicyImagePose;
        typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;

        std::string frame_id_;
        ros::Timer vis_timer_, dynamic_timer_;
        SynchronizerImagePose sync_image_pose_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
        std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
        std::shared_ptr<ros::Publisher> map_local_pub_, map_local_inflate_pub_, map_global_pub_, map_global_inflate_pub_,
                                        unknown_pub_, update_range_pub_, depth_pub_, frontiers_pub_;
        std::shared_ptr<ros::Publisher> dynamic_pub_;
        std::shared_ptr<ros::Publisher> debug_1_pub_, debug_2_pub_;


        void depthPose2Cloud(const cv::Mat &depthImg, pcl::PointCloud<pcl::PointXYZ>::Ptr &rayCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &obCloud);
        void updateDynamic(std::shared_ptr<ObstacleUtil> &ob);
       
        void visCallback(const ros::TimerEvent& event);
        void depthCallback(const sensor_msgs::ImageConstPtr &msgDepth);
        void depthPoseCallback(const sensor_msgs::ImageConstPtr &msgDepth, const geometry_msgs::PoseStampedConstPtr &msgOdom);
        void pubDynamic();
        void dynamicCallback(const ros::TimerEvent& event);

        void addDynamicObjectMarker(DynamicObject &obj);
        void deleteDynamicObjectMarkers();
        void updateDynamicObjectMarkers();
        
        void publishMapGlobal();
        void publishMapLocal();
        void publishUpdateRange();
        void publishUnknown();

        void pubFrontiers();
        void deleteMarkerArray(visualization_msgs::MarkerArray &mk_array);
        Eigen::Vector4d getColor(const int& h, double alpha) ;

    public:
        MapManager();
        ~MapManager();
        
        std::shared_ptr<Frontier> frontier_;

        void init(ros::NodeHandle &nh);

        void getDynamicInfo(std::vector<Eigen::Matrix<double, 9, 1>> &ob_dynamics);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif
