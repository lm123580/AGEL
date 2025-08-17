#include "kalman_filter.h"

namespace AGEL
{
    KalmanFilter::KalmanFilter()
    {
        isInited_ = false;

        mx_.setZero();

        mI_ = Eigen::MatrixXd::Identity(6, 6);

        mP_ <<  1, 0.0, 0.0, 0.6, 0.0, 0.0,
                0.0, 1, 0.0, 0.0, 0.6, 0.0,
                0.0, 0.0, 1, 0.0, 0.0, 0.6,
                0.6, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.6, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.6, 0.0, 0.0, 1.0;

        mQ_.setZero();
        mQ_.diagonal() << 0.1, 0.1, 0.1, 0.05, 0.05, 0.05;

        mR_.setZero();
        mR_.diagonal() << 0.1, 0.1, 0.1, 1, 1, 1;

        mH_ = Eigen::MatrixXd::Identity(6, 6);

        mF_ = Eigen::MatrixXd::Identity(6, 6);
    }


    void KalmanFilter::updateMF(double delta_T)
    {
        // mF_ <<  1.0, 0.0, 0.0, delta_T, 0.0, 0.0,
        //         0.0, 1.0, 0.0, 0.0, delta_T, 0.0,
        //         0.0, 0.0, 1.0, 0.0, 0.0, delta_T,
        //         0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        //         0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        //         0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        mF_.coeffRef(0, 3) = delta_T;
        mF_.coeffRef(1, 4) = delta_T;
        mF_.coeffRef(2, 5) = delta_T;
    }


    void KalmanFilter::init(Eigen::Vector3d &measurement, ros::Time &time)
    {
        isInited_ = true;

        mx_ << measurement[0], measurement[1], measurement[2], 0, 0, 0;

        T_ = time;
    }


    void KalmanFilter::update(Eigen::Vector3d &measurement, ros::Time &time)
    {
        if (!isInited())
        {
            init(measurement, time);

            return ;
        }


        double delta_T = (time - T_).toSec();
        updateMF(delta_T);
        T_ = time;

        Eigen::Vector3d vel = (measurement - mx_.head<3>()) / delta_T;
        Eigen::Matrix<double, 6, 1> zt;
        zt.block<3, 1>(0, 0) = measurement;
        zt.block<3, 1>(3, 0) = vel;

        mx_ = mF_ * mx_;
        mP_ = mF_ * mP_ * mF_.transpose() + mQ_;
        
        Eigen::Matrix<double,6,6> S = mH_ * mP_ * mH_.transpose() + mR_;
        
        Eigen::FullPivLU<Eigen::Matrix<double,6,6>> lu_decomposition(S);
        if(lu_decomposition.isInvertible())
        {
            mK_ = mP_ * mH_.transpose() * lu_decomposition.inverse();
            mx_ = mx_ + mK_ * (zt - mH_ * mx_);
            mP_ = (mI_ - mK_ * mH_) * mP_;
        }
        else
        {
            ROS_WARN("Kalman Filter Predict Failed !!!");
        }
    }


    Eigen::Matrix<double, 6, 1> KalmanFilter::predict(ros::Time &time)
    {
        double delta_T = (time - T_).toSec();
        updateMF(delta_T);

        Eigen::Matrix<double, 6, 1> predict_state = mF_ * mx_;

        return predict_state;
    }

    Eigen::Matrix<double, 6, 1> KalmanFilter::predict(double &delta_T)
    {
        updateMF(delta_T);

        Eigen::Matrix<double, 6, 1> predict_state = mF_ * mx_;

        return predict_state;
    }


    Eigen::Matrix<double, 6, 1> KalmanFilter::getState()
    {
        return mx_;
    }


    Eigen::Vector3d KalmanFilter::getPos()
    {
        return mx_.head<3>();
    }


    Eigen::Vector3d KalmanFilter::getVel()
    {
        return mx_.tail<3>();
    }


    bool KalmanFilter::isInited()
    {
        return isInited_;
    }
}
