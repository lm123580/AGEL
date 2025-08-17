#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "uniform_bspline.h"


AGEL::UniformBspline  b_spline;

visualization_msgs::Marker waypoints_mk, time_traj_mk, arc_traj_mk;
ros::Publisher waypoints_pub_, time_traj_pub_, arc_traj_pub_;

void visCallback(const ros::TimerEvent& event)
{
    time_traj_pub_.publish(time_traj_mk);
    arc_traj_pub_.publish(arc_traj_mk);
    waypoints_pub_.publish(waypoints_mk);
}

    // double NonUniformBspline::getLength(const double &res) {
    //     double length = 0.0;
    //     double dur = getTimeSum();
    //     Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
    //     for (double t = res; t <= dur + 1e-4; t += res) 
    //     {
    //         p_n = evaluateDeBoorT(t);
    //         length += (p_n - p_l).norm();
    //         p_l = p_n;
    //     }
    //     return length;
    // }                                       const int &degree, Eigen::MatrixXd &ctrl_pts) 


void fillBasicInfo( const Eigen::Vector3d& scale, const Eigen::Vector4d& color, const std::string& ns, 
                    const int& id, const int& shape, visualization_msgs::Marker& mk)
{
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.id = id;
    mk.ns = ns;
    mk.type = shape;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = scale[0];
    mk.scale.y = scale[1];
    mk.scale.z = scale[2];

    mk.lifetime = ros::Duration(0.1);
}


void fillGeometryInfo(visualization_msgs::Marker &mk, const std::vector<Eigen::Vector3d> &list)
{
    geometry_msgs::Point pt;

    for (int i = 0; i < (int)(list.size()); i++)
    {
        pt.x = list[i][0];
        pt.y = list[i][1];
        pt.z = list[i][2];
        mk.points.push_back(pt);
    }
}


void fill_waypoints(visualization_msgs::Marker &mk, std::vector<Eigen::Vector3d> &list)
{
    fillBasicInfo(Eigen::Vector3d(0.1, 0.1, 0.1), Eigen::Vector4d(0, 0, 1, 1), "waypoints", 0, visualization_msgs::Marker::SPHERE_LIST, mk);
    fillGeometryInfo(mk, list);
}

void fill_bspline_traj(visualization_msgs::Marker &mk, std::vector<Eigen::Vector3d> &list, int id)
{
    if (id == 1)
    {
        fillBasicInfo(Eigen::Vector3d(0.05, 0.05, 0.05), Eigen::Vector4d(1, 0, 0, 0.7), "time_traj", 1, visualization_msgs::Marker::SPHERE_LIST, mk);
    }
    else
    {
        fillBasicInfo(Eigen::Vector3d(0.05, 0.05, 0.05), Eigen::Vector4d(0, 1, 0, 0.7), "arc_traj", 1, visualization_msgs::Marker::SPHERE_LIST, mk);
    }

    fillGeometryInfo(mk, list);
}

void get_bernoulli(std::vector<Eigen::Vector3d> &waypoints)
{
    double r, x, y, z;

    std::vector<Eigen::Vector3d> tmp_waypoints;
    for (double theta = M_PI_4; theta >= 0; theta -= 0.05)
    {
        r = 4 * std::sqrt(std::cos(2 * theta));

        x = r * cos(theta);
        y = r * sin(theta);
        z = 1;

        // std::cout << theta << " " << x << " " << y << std::endl;
        tmp_waypoints.push_back(Eigen::Vector3d(x, y, z));
    }

    int tmp_size = tmp_waypoints.size();
    
    for (int i = 0; i < tmp_size; i++)
    {
        waypoints.push_back(Eigen::Vector3d(tmp_waypoints[i][0], tmp_waypoints[i][1], tmp_waypoints[i][2]));
    }

    for (int i = tmp_size - 1; i >= 0; i--)
    {
        waypoints.push_back(Eigen::Vector3d(tmp_waypoints[i][0], -tmp_waypoints[i][1], tmp_waypoints[i][2]));
    }

    for (int i = 0; i < tmp_size; i++)
    {
        waypoints.push_back(Eigen::Vector3d(-tmp_waypoints[i][0], tmp_waypoints[i][1], tmp_waypoints[i][2]));
    }

    for (int i = tmp_size - 1; i >= 0; i--)
    {
        waypoints.push_back(Eigen::Vector3d(-tmp_waypoints[i][0], -tmp_waypoints[i][1], tmp_waypoints[i][2]));
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "b_node");
    ros::NodeHandle nh("~");

    double x = 0, y = 0, z = 0;
    std::vector<Eigen::Vector3d> waypoints;
    for (int i = 0; i <= 20; i++)
    {
        x = 2 * M_PI * (double)i / 20.0;
        y = sin(x) * 2;
        z = (double)i / 10;

        Eigen::Vector3d waypoint(x, y, z);

        waypoints.push_back(waypoint);
    }
    ros::Time t1 = ros::Time::now();

    // std::vector<Eigen::Vector3d> waypoints;
    // get_bernoulli(waypoints);


    std::vector<Eigen::Vector3d> start_end_derivative;
    start_end_derivative.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    start_end_derivative.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    start_end_derivative.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    start_end_derivative.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));

    Eigen::MatrixXd ctrl_pts;
    b_spline.parameterizeToBspline(10.0, waypoints, start_end_derivative, 3, ctrl_pts);     // B样条插值拟合
    b_spline.setUniformBspline(ctrl_pts, 3, 1.0);

    ros::Time t2 = ros::Time::now();

    // fill_waypoints(waypoints_mk, waypoints);
    std::vector<Eigen::Vector3d> sample_points;                                             // 采样填充时间参数化B样条曲线
    double res = 0.01;
    b_spline.getSamplePoints(sample_points, res);
    double sample_num = 3000;
    std::string save_name = "./data/time_traj.txt";
    b_spline.saveInfo(save_name, sample_num);
    fill_bspline_traj(time_traj_mk, sample_points, 1);

    ros::Time t3 = ros::Time::now();

    waypoints = b_spline.parameterizeToArcBspline(30);                                      // 弧长参数化B样条
    b_spline.initDerivativeCtrlPts();

    ros::Time t4 = ros::Time::now();

    fill_waypoints(waypoints_mk, waypoints);                                                // 填充航点
    
    ros::Time t5 = ros::Time::now();
    
    b_spline.getSamplePoints(sample_points, res);                                           // 采样填充弧长参数化B样条曲线
    save_name = "./data/arc_traj.txt";
    b_spline.saveInfo(save_name, sample_num);
    fill_bspline_traj(arc_traj_mk, sample_points, 2);
    
    ros::Time t6 = ros::Time::now();

    ros::Timer vis_timer = nh.createTimer(ros::Duration(0.1), &visCallback);
    waypoints_pub_ = nh.advertise<visualization_msgs::Marker>("/visualiztion/waypoints", 5);
    arc_traj_pub_ = nh.advertise<visualization_msgs::Marker>("/visualiztion/arc_bspline_traj", 5);
    time_traj_pub_ = nh.advertise<visualization_msgs::Marker>("/visualiztion/time_bspline_traj", 5);

    ROS_INFO("time: %.10f  %.10f  %.10f  %.10f %10f", (t2-t1).toSec(), (t3-t2).toSec(), (t4-t3).toSec(), (t5-t4).toSec(), (t6-t1).toSec());

    ros::spin();

    ros::shutdown();


    return 0;
}