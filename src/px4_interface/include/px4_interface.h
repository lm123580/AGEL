#ifndef _PX4_INTERFACE_H_
#define _PX4_INTERFACE_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include <mutex>

#include <Eigen/Eigen>


namespace AGEL
{
    class Px4Interface
    {
    private:
        std::mutex local_pose_mutex, local_vel_mutex, local_rate_mutex, acc_mutex;
        std::shared_ptr<ros::Subscriber> pose_sub_, vel_sub_, acc_sub_, rate_sub_, px4_state_sub_;
        std::shared_ptr<ros::Publisher> pos_target_pub_, att_target_pub_;
        std::shared_ptr<ros::ServiceClient> px4_mode_srv_, px4_arm_srv_;

        double yaw_;
        Eigen::Vector3d pos_, vel_, acc_, rate_;
        Eigen::Quaterniond quat_;

        bool px4_armed_;
        std::string px4_mode_;
        ros::Time vel_stamp_, pos_cmd_time_, rate_cmd_time_;

        void local_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg);
        void local_vel_callback(const geometry_msgs::TwistStampedConstPtr &msg);
        void acc_callback(const sensor_msgs::ImuConstPtr &msg);
        void local_rate_callback(const geometry_msgs::TwistStampedConstPtr &msg);
        void px4_state_callback(const mavros_msgs::StateConstPtr &msg);

    public:
        Px4Interface(ros::NodeHandle &nh);
        ~Px4Interface();

        void set_rate_with_trust(double &rx, double &ry, double &rz, double &thrust);
        void set_attitude_with_trust(Eigen::Quaterniond &q, double &thrust);
        void set_pos(double &x, double &y, double &z, double &yaw);
        void set_pos_with_yaw_rate(double &x, double &y, double &z, double &yaw_rate);
        void arm();
        void disarm();
        void set_px4_mode(std::string mode);

        double get_yaw() { return yaw_; }
        Eigen::Vector3d get_pos() { return pos_; }
        Eigen::Vector3d get_vel() { return vel_; }
        Eigen::Vector3d get_acc() { return acc_; }
        Eigen::Vector3d get_rate() { return rate_; }
        std::string get_mode() { return px4_mode_; }

        inline Eigen::Quaterniond get_quat() { return quat_; };

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}


#endif