#include "px4_interface.h"

namespace AGEL
{
    Px4Interface::Px4Interface(ros::NodeHandle &nh)
    {
        px4_armed_ = false;

        pose_sub_ = std::make_shared<ros::Subscriber>(nh.subscribe("/iris/pose", 1, &Px4Interface::local_pose_callback, this));
        vel_sub_ = std::make_shared<ros::Subscriber>(nh.subscribe("/iris/vel", 1, &Px4Interface::local_vel_callback, this));
        acc_sub_ = std::make_shared<ros::Subscriber>(nh.subscribe("/mavros/imu/data_raw", 1, &Px4Interface::acc_callback, this));
        rate_sub_ = std::make_shared<ros::Subscriber>(nh.subscribe("/mavros/local_position/velocity_body", 1, &Px4Interface::local_rate_callback, this));
        px4_state_sub_ = std::make_shared<ros::Subscriber>(nh.subscribe("/mavros/state", 1, &Px4Interface::px4_state_callback, this));

        pos_target_pub_ = std::make_shared<ros::Publisher>(nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",1));
        att_target_pub_ = std::make_shared<ros::Publisher>(nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1));

        px4_mode_srv_ = std::make_shared<ros::ServiceClient>(nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode"));
        px4_arm_srv_ = std::make_shared<ros::ServiceClient>(nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming"));
    
        pos_cmd_time_ = ros::Time::now();
        rate_cmd_time_ = ros::Time::now();
    }


    Px4Interface::~Px4Interface() = default;


    void Px4Interface::set_rate_with_trust(double &rx, double &ry, double &rz , double &thrust) 
    {
        // rate_cmd_time_ = ros::Time::now();

        // if ((rate_cmd_time_ - pos_cmd_time_).toSec() * 1000 < 50.0)
        // {
        //     return ;
        // }

        mavros_msgs::AttitudeTarget cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "FCU";
        cmd.body_rate.x = rx;
        cmd.body_rate.y = ry;
        cmd.body_rate.z = rz;
        cmd.thrust = thrust;
        cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        att_target_pub_->publish(cmd);
    }


    void Px4Interface::set_attitude_with_trust(Eigen::Quaterniond &q, double &thrust) 
    {
        mavros_msgs::AttitudeTarget cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "FCU";
        cmd.orientation.x = q.x();
        cmd.orientation.y = q.y();
        cmd.orientation.z = q.z();
        cmd.orientation.w = q.w();
        cmd.thrust = thrust;
        cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        att_target_pub_->publish(cmd);
    }


    void Px4Interface::set_pos(double &x, double &y, double &z, double &yaw) 
    {        
        mavros_msgs::PositionTarget cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "FCU";
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.position.x = x;
        cmd.position.y = y;
        cmd.position.z = z;
        cmd.yaw = yaw;
        cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_VX + mavros_msgs::PositionTarget::IGNORE_VY + mavros_msgs::PositionTarget::IGNORE_VZ +
                        mavros_msgs::PositionTarget::IGNORE_AFX + mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ +
                        mavros_msgs::PositionTarget::FORCE + mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        pos_target_pub_->publish(cmd);
    }


    void Px4Interface::set_pos_with_yaw_rate(double &x, double &y, double &z, double &yaw_rate) 
    {        
        mavros_msgs::PositionTarget cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "FCU";
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.position.x = x;
        cmd.position.y = y;
        cmd.position.z = z;
        cmd.yaw_rate = yaw_rate;
        cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_VX
                    + mavros_msgs::PositionTarget::IGNORE_VY
                    + mavros_msgs::PositionTarget::IGNORE_VZ
                    + mavros_msgs::PositionTarget::IGNORE_AFX
                    + mavros_msgs::PositionTarget::IGNORE_AFY
                    + mavros_msgs::PositionTarget::IGNORE_AFZ
                    + mavros_msgs::PositionTarget::FORCE
                    + mavros_msgs::PositionTarget::IGNORE_YAW;  // 忽略yaw，使用yaw_rate
        pos_target_pub_->publish(cmd);
    }


    void Px4Interface::arm() 
    {
        mavros_msgs::CommandBool req;
        req.request.value = true;

        if (px4_arm_srv_->call(req)) ROS_INFO("Arm");
        else                         ROS_WARN("Vehicle arming failed");
    }


    void Px4Interface::disarm() 
    {
        mavros_msgs::CommandBool req;
        req.request.value = false;

        if (px4_arm_srv_->call(req)) ROS_INFO("Disarm");
        else                         ROS_WARN("Vehicle disarming failed");
    }


    void Px4Interface::set_px4_mode(std::string mode) 
    {
        mavros_msgs::SetMode req;
        req.request.custom_mode = mode;

        px4_mode_srv_->call(req);
        // if (px4_mode_srv_->call(req))    ROS_INFO("Set mode: %s", mode.c_str());
        // else                             ROS_WARN("Failed to set mode: %s", mode.c_str());
    }


    void Px4Interface::local_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg) 
    {
        std::unique_lock<std::mutex> lock(local_pose_mutex);

        pos_.x() = msg->pose.position.x;
        pos_.y() = msg->pose.position.y;
        pos_.z() = msg->pose.position.z;
        quat_.x() = msg->pose.orientation.x;
        quat_.y() = msg->pose.orientation.y;
        quat_.z() = msg->pose.orientation.z;
        quat_.w() = msg->pose.orientation.w;

        double qx = quat_.x();
        double qy = quat_.y();
        double qz = quat_.z();
        double qw = quat_.w();
        double tmpx = 1. - 2. * (qy * qy + qz * qz);
        double tmpy = 2. * (qw * qz + qx * qy);
        yaw_  = std::atan2(tmpy, tmpx);
    }


    void Px4Interface::local_vel_callback(const geometry_msgs::TwistStampedConstPtr &msg) 
    {
        std::unique_lock<std::mutex> lock(local_vel_mutex);

        vel_.x() = msg->twist.linear.x;
        vel_.y() = msg->twist.linear.y;
        vel_.z() = msg->twist.linear.z;
        vel_stamp_ = msg->header.stamp;
    }


    void Px4Interface::acc_callback(const sensor_msgs::ImuConstPtr &msg) 
    {
        std::unique_lock<std::mutex> lock(acc_mutex);

        acc_.x() = msg->linear_acceleration.x;
        acc_.y() = msg->linear_acceleration.y;
        acc_.z() = msg->linear_acceleration.z;
    }


    void Px4Interface::local_rate_callback(const geometry_msgs::TwistStampedConstPtr &msg) 
    {
        std::unique_lock<std::mutex> lock(local_rate_mutex);

        rate_.x() = msg->twist.angular.x;
        rate_.y() = msg->twist.angular.y;
        rate_.z() = msg->twist.angular.z;
    }


    void Px4Interface::px4_state_callback(const mavros_msgs::StateConstPtr &msg) 
    {
        px4_armed_ = msg->armed;
        px4_mode_ = msg->mode;
    }

    
}
