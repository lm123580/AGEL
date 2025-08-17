#ifndef CAMERA_UTIL_H_
#define CAMERA_UTIL_H_

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace AGEL
{
    /*
    *   
    *   相机相关参数、位姿和视角
    *
    */
    class CameraUtil
    {
    public:
        Eigen::Matrix4d pose_, Twc_;
        int img_rows_, img_cols_;
        double cx_, cy_, fx_, fy_;
        double min_dist_, max_dist_;

    private:
        double hFOV_, vFOV_;
        // Eigen::Matrix4d Tcb_, Tbc_;
        Eigen::Matrix4d Tbc_;
        Eigen::Vector3d n_z_c_, n_top_c_, n_bottom_c_, n_left_c_, n_right_c_;   // 相机四个平面法向量（相机坐标系下）
        Eigen::Vector3d n_z_w_, n_top_w_, n_bottom_w_, n_left_w_, n_right_w_;   // 相机四个平面法向量（世界坐标系下）

        void initNorm();                                              // 更新相机平面法向量
    
    public:
        CameraUtil() = delete;
        CameraUtil(ros::NodeHandle &nh);
    
        void updatePose(const Eigen::Matrix4d &pose);           // 更新相机位姿
        bool insideFOV(const Eigen::Vector3d &point);           // 判断某点是否在相机视角范围内
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif
