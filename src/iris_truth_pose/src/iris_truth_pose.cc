#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>

#include <vector>
#include <string>

ros::Publisher iris_pose_pub_, iris_vel_pub_;

geometry_msgs::PoseStamped  iris_pose_;
geometry_msgs::TwistStamped iris_vel_;

std::string iris_name = "iris";


void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    auto it = std::find(msg->name.begin(), msg->name.end(), iris_name);
    int idx = std::distance(msg->name.begin(), it);

    // 位姿
    iris_pose_.header.stamp = ros::Time::now();
    iris_pose_.header.frame_id = "map";
    iris_pose_.pose = msg->pose[idx];
    
    // 速度
    iris_vel_.header.stamp = ros::Time::now();
    iris_vel_.header.frame_id = "map";
    iris_vel_.twist = msg->twist[idx];
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "iris_truth_pose");
    ros::NodeHandle nh;

    // 等待ros::Time初始化成功
    while (!ros::Time::isValid()) 
    {
        ros::Duration(0.05).sleep(); 
    }

    iris_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/gazebo/iris/pose", 1);
    iris_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/gazebo/iris/vel", 1);

    ros::Subscriber model_state_sub = nh.subscribe("/gazebo/model_states", 1, modelStateCallback);

    ros::Rate rate(100);

    while(ros::ok())
    {
        iris_pose_pub_.publish(iris_pose_);
        iris_vel_pub_.publish(iris_vel_);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}