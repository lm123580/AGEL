#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>

namespace AGEL
{
    class KalmanFilter
    {
    private:
        bool isInited_;

        ros::Time T_;

        Eigen::Matrix<double, 6, 1> mx_;
        Eigen::Matrix<double, 6, 6> mR_;
        Eigen::Matrix<double, 6, 6> mH_;
        Eigen::Matrix<double, 6, 6> mK_;
        Eigen::Matrix<double, 6, 6> mP_, mQ_, mF_, mI_;
        
        void updateMF(double delta_T);

    public:
        bool isInited();
        void init(Eigen::Vector3d &measurement, ros::Time &time);
        void update(Eigen::Vector3d &measurement, ros::Time &time);
        Eigen::Matrix<double, 6, 1> predict(ros::Time &time);
        Eigen::Matrix<double, 6, 1> predict(double &detal_T);
        Eigen::Matrix<double, 6, 1> getState();
        Eigen::Vector3d getPos();
        Eigen::Vector3d getVel();

        KalmanFilter();

        ~KalmanFilter()
        {
        }
        
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}

#endif