#include "camera_util.h"

namespace AGEL
{
    CameraUtil::CameraUtil(ros::NodeHandle &nh)
    {
        nh.param("camera_util/img_cols", img_cols_, 640);
        nh.param("camera_util/img_rows", img_rows_, 480);
        nh.param("camera_util/fx", fx_, 565.6008952774197);
        nh.param("camera_util/fy", fy_, 565.6008952774197);
        nh.param("camera_util/cx", cx_, 320.5);
        nh.param("camera_util/cy", cy_, 240.5);
        nh.param("camera_util/min_dist", min_dist_, 0.2);
        nh.param("camera_util/max_dist", max_dist_, 5.0);

        Tbc_ << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        
        hFOV_ = 2 * std::atan(img_cols_ / (2 * fx_)) * (180.0 / M_PI);          // 计算相机视场
        vFOV_ = 2 * std::atan(img_rows_ / (2 * fy_)) * (180.0 / M_PI);
        
        initNorm();                                                             // 初始化平面法向量
    }

    void CameraUtil::initNorm()
    {
        double left_angle = (cx_ / img_cols_) * hFOV_ * M_PI / 180.0;
        double right_angle = (1.0 - cx_ / img_cols_) * hFOV_ * M_PI / 180.0;
        double top_angle = (cy_ / img_rows_) * vFOV_ * M_PI / 180.0;
        double bottom_angle = (1.0 - cy_ / img_rows_) * vFOV_ * M_PI / 180.0;

        // std::cout << hFOV_ << std::endl;
        // std::cout << vFOV_ << std::endl;

        n_z_c_ << 0.0, 0.0, 1.0;
        n_top_c_ << 0.0, sin(M_PI_2 - top_angle), cos(M_PI_2 - top_angle);
        n_bottom_c_ << 0.0, -sin(M_PI_2 - bottom_angle), cos(M_PI_2 - bottom_angle);
        n_left_c_ << sin(M_PI_2 - left_angle), 0.0, cos(M_PI_2 - left_angle);
        n_right_c_ << -sin(M_PI_2 - right_angle), 0.0, cos(M_PI_2 - right_angle);
    }


    // 更新相机在世界坐标系的位姿
    void CameraUtil::updatePose(const Eigen::Matrix4d &pose)
    {
        pose_ = pose;                                                           // pose 是 Twb

        Twc_ = pose * Tbc_;

        Eigen::Matrix3d Rwc = Twc_.block<3, 3>(0, 0);

        n_z_w_ = Rwc * n_z_c_;
        n_top_w_ = Rwc * n_top_c_;
        n_bottom_w_ = Rwc * n_bottom_c_;
        n_left_w_ = Rwc * n_left_c_;
        n_right_w_ = Rwc * n_right_c_;
    }


    bool CameraUtil::insideFOV(const Eigen::Vector3d &point)
    {
        Eigen::Vector3d dir = point - pose_.block<3, 1>(0, 3);

        double distance = dir.dot(n_z_w_) / (n_z_w_.norm());

        // 判断距离
        if(distance > max_dist_ || distance < min_dist_)
        {
            return false;
        }

        // 判断方向
        return (dir.dot(n_top_w_) > 0.0 && dir.dot(n_bottom_w_) > 0.0 && dir.dot(n_left_w_) > 0.0 && dir.dot(n_right_w_) > 0.0);
    }
}