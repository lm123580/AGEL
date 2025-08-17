#include "motion_manager.h"
#include "map_manager.h"
#include "sdf_map.h"
#include "safe_flight_corridor.h"
#include "astar.h"
#include "uniform_bspline.h"
#include "bspline_opt.h"
#include "px4_interface.h"
#include "quadrotor_dynamic.h"
#include "mpcc.h"

#include "frontier.h"

namespace xxx_xxx
{
    MotionManager::MotionManager() = default;
    
    MotionManager::~MotionManager() = default;
    
    void MotionManager::init(ros::NodeHandle& nh, std::shared_ptr<MapManager> &map_manager, std::shared_ptr<Px4Interface> &px4_interface)
    {
        nh.param("local_motion/hover_ratio", haver_ratio_, 0.710);
        
        nh.param("local_motion/cost_w_lag",     cost_w_[0],  40.0);
        nh.param("local_motion/cost_w_contour", cost_w_[1],  0.85);
        nh.param("local_motion/cost_w_progress",cost_w_[2],  8.00);
        nh.param("local_motion/cost_w_r_yaw",   cost_w_[3],  0.50);
        nh.param("local_motion/cost_w_v_yaw",   cost_w_[4],  2.00);
        nh.param("local_motion/cost_w_u_rate",  cost_w_[5],  1.00);
        nh.param("local_motion/cost_w_u_trust", cost_w_[6],  30.0);
        nh.param("local_motion/cost_w_sfc",     cost_w_[7],  3.00);
        nh.param("local_motion/cost_w_dynamic", cost_w_[8],  5.00);
        
        map_manager_ = map_manager;
        
        px4_interface_ = px4_interface;

        sfc_.reset(new SFC(nh, map_manager_->map_));

        astar_.reset(new Astar(nh, map_manager_->map_));

        bspline_.reset(new UniformBspline);

        bspline_opt_.reset(new BsplineOptimizer(nh));

        quadrotor_dynamic_.reset(new QuadrotorDynamic(haver_ratio_));

        mpcc_.reset(new MPCC(quadrotor_dynamic_));

        bspline_exec_ = nullptr;
        
        last_u_ << 0.0, 0.0, 0.0, haver_ratio_;

        for (int i = 0; i < MPCC::n_step_; i++)
        {
            u_predict_.block(i, 0, 1, 4) = last_u_.transpose();
        }

        for (int i = 0; i < 11; i++)
        {
            t_index_[i] = 0.1 * i;
        }

        passed_path_pub_ = std::make_shared<ros::Publisher>(nh.advertise<nav_msgs::Path>("/motion/passed_path", 1));
        predict_state_pub_ = std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("/motion/predict_state", 1));
        astar_pub_ = std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("/motion/astar_traj", 1));
        time_bspline_pub_ = std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("/motion/time_bspline_traj", 1));
        arc_bspline_pub_ = std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("/motion/arc_bspline_traj", 1));
    }
    

    bool MotionManager::globalPlanning(Eigen::Vector3d& goal, const bool is_replan)
    {
        // ros::Time tt1 = ros::Time::now();
        ros::Time t1, t2, t3, t4, t5, t6;

        t1 = ros::Time::now();

        bool have_path = false;

        bool replan_flag = is_replan;

        if (global_waypoints_.size() < 2 && is_replan == true)
        {
            replan_flag = false;
        }

        std::vector<Eigen::Vector3d> global_waypoints;
        if (!replan_flag || global_waypoints_.size() < 2)
        {
            Eigen::Vector3d pos = px4_interface_->get_pos();
            if (astar_->globalPlan(pos, goal))
            {
                astar_->getPath(global_waypoints);
                have_path = true;
        std::cout << "22" << std::endl;

            }
            else
            {
                std::cout << "First palnning: There is no path to goal" << std::endl;
            }
        }
        else
        {
            if (astar_->replan(px4_interface_->get_pos(), goal))
            {
                astar_->getPath(global_waypoints);
                have_path = true;
            }
            else
            {
                if (astar_->globalPlan(px4_interface_->get_pos(), goal))
                {
                    astar_->getPath(global_waypoints);
                    have_path = true;
                }
                else
                {
                    std::cout << "Re palnning: There is no path to goal" << std::endl;
                }
            }
        }

        t2 = ros::Time::now();


        std::cout << "Astar path size is: " << global_waypoints.size() << std::endl;
        // ros::Time tt2 = ros::Time::now();

        // std::cout << "astar time is: " << (tt2 - tt1).toSec() * 1000.0 << "ms" << std::endl;
        
        if (!have_path)
        {
            arc_ts_ = 0.0;
            arc_length_ = 0.0;
            
            bspline_mutex_.lock();
            bspline_exec_ = nullptr;
            bspline_mutex_.unlock();

            global_waypoints_ = std::vector<Eigen::Vector3d>();
            time_bspline_waypoints_ = std::vector<Eigen::Vector3d>();
            arc_bspline_waypoints_ = std::vector<Eigen::Vector3d>();

            predict_state_pos_ = std::vector<Eigen::Vector3d>();
            predict_state_Q_ = std::vector<Eigen::Vector4d>();

            return false;
        }
        
        // ros::Time t1 = ros::Time::now();

        // B样条拟合
        double res = 0.01;
        double rough_length = astar_->pathLength();

        int astar_size = global_waypoints.size();
        Eigen::Vector3d start_pos = global_waypoints[0];
        Eigen::Vector3d start_vel = Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d start_acc = Eigen::Vector3d(0.0, 0.0, 0.0);

        Eigen::Vector3d end_pos = global_waypoints[astar_size - 1];
        Eigen::Vector3d end_vel = Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d end_acc = Eigen::Vector3d(0.0, 0.0, 0.0);

        std::vector<Eigen::Vector3d> start_end_derivative;
        start_end_derivative.push_back(start_vel);
        start_end_derivative.push_back(end_vel);
        start_end_derivative.push_back(start_acc);
        start_end_derivative.push_back(end_acc);
        Eigen::MatrixXd ctrl_pts;
        bspline_->parameterizeToBspline(5, global_waypoints, start_end_derivative, 3, ctrl_pts);
        double dt = rough_length / (astar_size - 1);
        bspline_->setUniformBspline(ctrl_pts, 3, dt);
        bspline_->getSamplePoints(time_bspline_waypoints_, res);

        t3 = ros::Time::now();


        // B样条弧长参数化
        ctrl_pts = bspline_->getControlPoints();
        start_vel = (global_waypoints[1] - global_waypoints[0]).normalized();
        end_vel = (global_waypoints[astar_size - 1] - global_waypoints[astar_size - 2]).normalized();
        bspline_opt_->setBoundaryStates(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc);
        bspline_opt_->solver(ctrl_pts, dt);
        // ros::Time t2 = ros::Time::now();
        // std::cout << "Bspline generate and opt time is : " << (t2 - t1).toSec() * 1000.0 << "ms" << std::endl;

        t4 = ros::Time::now();

        // 缩短路径
        std::vector<Eigen::Vector3d> short_path;
        splitPath(global_waypoints, short_path);

        // 设置B样条
        // bspline_->setUniformBspline(ctrl_pts, 3, dt);
        int short_ctrl_pts_num = short_path.size() + 3 - 1;
        Eigen::MatrixXd short_ctrl_pts = ctrl_pts.block(0, 0, short_ctrl_pts_num, ctrl_pts.cols());
        bspline_->setUniformBspline(short_ctrl_pts, 3, dt);

        // 初始化B样条的导数
        bspline_->initDerivativeCtrlPts();

        // 获得可执行的B样条
        bspline_mutex_.lock();
        bspline_exec_.reset(new UniformBspline(*bspline_));
        bspline_mutex_.unlock();

        // 离散点采样
        bspline_exec_->getSamplePoints(arc_bspline_waypoints_, res);

        global_waypoints_ = std::move(global_waypoints);

        t5 = ros::Time::now();

        // 生成安全飞行走廊
        // ros::Time t3 = ros::Time::now();
        sfc_->generateSFC(short_path);
        sfc_->visCorridor();
        sfc_->getCorridorPolys(corridor_polys_);
        sfc_->getpath2hPloyIdx(path2hPloy_idx_);

        t6 = ros::Time::now();

        pubAstarTraj(global_waypoints_);
        pubReferTimeBspline(time_bspline_waypoints_);
        pubReferArcBspline(arc_bspline_waypoints_);
        // ros::Time t4 = ros::Time::now();
        // std::cout << "sfc time is : " << (t4 - t3).toSec() * 1000.0 << "ms" << std::endl;
        // std::cout << "Total time is : " << (t4 - tt1).toSec() * 1000.0 << "ms" << std::endl;

        search_path_info_.num++;
        search_path_info_.total_time += ((t2 - t1).toSec() * 1000.0);            
        global_opt_info_.num++;
        global_opt_info_.total_time += ((t4 - t3).toSec() * 1000.0);  
        sfc_info_.num++;
        sfc_info_.total_time += ((t6 - t5).toSec() * 1000.0);

        print_num_++;
        if (print_num_ % 100 == 0) {
            std::cout << "全局规划时间：" << (search_path_info_.total_time / search_path_info_.num) << std::endl;
            std::cout << "全局优化时间：" << (global_opt_info_.total_time / global_opt_info_.num) << std::endl;
            std::cout << "安全飞行走廊：" << (sfc_info_.total_time / sfc_info_.num) << std::endl;
            print_num_ = 0;
        }

        return true;
    }


    double MotionManager::getGlobalStartYaw()
    {
        if (bspline_exec_ == nullptr)
        {
            return 1000000.0;
        }

        // 获取B样条相关参数
        Eigen::MatrixXd ctrl_pts;
        ctrl_pts = bspline_exec_->getControlPoints();
        Eigen::VectorXd start_vel;
        double t_start = 0.05;
        
        bspline_exec_->getBsplineValueFast(t_start, ctrl_pts, 3, start_vel);
        
        double start_yaw = std::atan2(start_vel[1], start_vel[0]);

        return start_yaw;
    }


    void MotionManager::localPlanning()
    {
        if (bspline_exec_ == nullptr || global_waypoints_.size() < 2)
        {
            return ;
        }
        
        // ros::Time local_start_time = ros::Time::now();
        // 获取B样条相关参数
        double arc_ts, arc_length;
        Eigen::MatrixXd ctrl_pts, v_ctrl_pts;
        
        bspline_mutex_.lock();

        ctrl_pts = bspline_exec_->getControlPoints();
        v_ctrl_pts = bspline_exec_->getVControlPoints();
        arc_ts = bspline_exec_->getKnotSpan();
        arc_length = bspline_exec_->getTimeSum();

        bspline_mutex_.unlock();

        // 安全飞行走廊
        std::vector<Eigen::MatrixXd> corridor_polys = corridor_polys_;
        std::vector<int> path2hPloy_idx = path2hPloy_idx_;

        // 动态物体
        std::vector<Eigen::Matrix<double, 9, 1>> ob_dynamic;
        map_manager_->getDynamicInfo(ob_dynamic);

        // 无人机当前状态
        Eigen::Matrix<double, 10, 1> state;
        state.block<3, 1>(0, 0) = px4_interface_->get_pos();
        state.block<3, 1>(3, 0) = px4_interface_->get_vel();
        state.block<4, 1>(6, 0) << px4_interface_->get_quat().w(), px4_interface_->get_quat().x(), px4_interface_->get_quat().y(), px4_interface_->get_quat().z();

        mpcc_->set_w(cost_w_);

        ros::Time t1, t2;

        t1 = ros::Time::now();
        // 求解最优输入
        mpcc_->solver(state, 
                      ctrl_pts,
                      v_ctrl_pts,
                      arc_ts,
                      arc_length,
                      corridor_polys,
                      path2hPloy_idx,
                      ob_dynamic,
                      last_u_,
                      u_predict_,
                      x_predict_,
                      t_index_);
        last_u_ = u_predict_.row(0);

        t2 = ros::Time::now();

        // 发布控制指令
        px4_interface_->set_rate_with_trust(last_u_[0], last_u_[1], last_u_[2], last_u_[3]);

        // 更新状态记录
        int step_size = MPCC::n_step_;
        predict_state_pos_.clear();
        predict_state_Q_.clear();
        for (int k = 0; k < step_size; k++)
        {
            Eigen::Vector3d p;
            Eigen::Vector4d q;
            p = x_predict_.block(k, 0, 1, 3).transpose();
            q = x_predict_.block(k, 6, 1, 4).transpose();
            predict_state_pos_.push_back(p);
            predict_state_Q_.push_back(q);
        }

        // ros::Time local_end_time = ros::Time::now();
        // std::cout << "Total time is : " << (local_end_time - local_start_time).toSec() * 1000.0 << "ms" << std::endl;

        pubPredictState(predict_state_pos_, predict_state_Q_);

        local_info_.num++;
        local_info_.total_time += ((t2 - t1).toSec() * 1000.0);
        print_num2_++;
        if (print_num2_ % 100 == 0)
        {
            std::cout << "局部运动规划时间：" << (local_info_.total_time / local_info_.num) << std::endl;
            print_num2_ = 0;
        }
    }


    void MotionManager::splitPath(const std::vector<Eigen::Vector3d> &path, std::vector<Eigen::Vector3d> &split_path)
    {
        split_path.clear();

        double path_length = 0.0;

        int path_size = path.size();
        split_path.push_back(path.front());
        for (int i = 1; i < path_size && path_length < 10.0; i++)
        {
            path_length += (path[i] - path[i-1]).norm();
            split_path.push_back(path[i]);
        }
    }


    void MotionManager::pathTimerCallback(const ros::TimerEvent& event)
    {

    }

    void MotionManager::deleteMarkerArray(visualization_msgs::MarkerArray &mk_array)
    {
        int mk_size = mk_array.markers.size();
        for (int i = 0; i < mk_size; i++)
        {
            mk_array.markers[i].action = visualization_msgs::Marker::DELETE;
        }
    }


    void MotionManager::pubPredictState(const std::vector<Eigen::Vector3d> &pos, const std::vector<Eigen::Vector4d> &Q)
    {

        // 删除上一次的标记
        deleteMarkerArray(predict_state_);
        predict_state_pub_->publish(predict_state_);
        predict_state_.markers.clear();

        visualization_msgs::Marker mk;

        mk.header.frame_id = "map";
        mk.header.stamp = ros::Time::now();
        mk.type = visualization_msgs::Marker::MESH_RESOURCE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.mesh_resource = "package://system_fsm/meshes/f250.dae";
        mk.scale.x = 1.0;
        mk.scale.y = 1.0;
        mk.scale.z = 1.0;
        mk.color.a = 1.0;

        int num = pos.size();
        for (int i = 0; i < num; i++)
        {
            mk.id = predict_state_.markers.size();

            mk.pose.position.x = pos[i].x();
            mk.pose.position.y = pos[i].y();
            mk.pose.position.z = pos[i].z();
            mk.pose.orientation.w = Q[i][0];
            mk.pose.orientation.x = Q[i][1];
            mk.pose.orientation.y = Q[i][2];
            mk.pose.orientation.z = Q[i][3];

            predict_state_.markers.push_back(mk);
        }

        // 发布新的标记
        predict_state_pub_->publish(predict_state_);
    }


    void MotionManager::pubReferTimeBspline(const std::vector<Eigen::Vector3d> &waypoints)
    {
        // 删除上一次的标记
        deleteMarkerArray(time_bspline_traj_);
        time_bspline_pub_->publish(time_bspline_traj_);
        time_bspline_traj_.markers.clear();

        geometry_msgs::Point p;
        visualization_msgs::Marker mk;

        mk.header.frame_id = "map";
        mk.header.stamp = ros::Time::now();
        mk.type = visualization_msgs::Marker::SPHERE_LIST;
        mk.action = visualization_msgs::Marker::ADD;
        mk.id = time_bspline_traj_.markers.size();
        mk.scale.x = 0.1;
        mk.scale.y = 0.1;
        mk.scale.z = 0.1;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
        mk.color.a = 0.7;
        mk.color.r = 1.000;
        mk.color.g = 0.000;
        mk.color.b = 0.000;

        for (auto &point: waypoints)
        {
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            mk.points.push_back(p);
        }
        time_bspline_traj_.markers.push_back(mk);

        // 发布新的标记
        time_bspline_pub_->publish(time_bspline_traj_);
    }


    void MotionManager::pubReferArcBspline(const std::vector<Eigen::Vector3d> &waypoints)
    {
        // 删除上一次的标记
        deleteMarkerArray(arc_bspline_traj_);
        arc_bspline_pub_->publish(arc_bspline_traj_);
        arc_bspline_traj_.markers.clear();

        geometry_msgs::Point p;
        visualization_msgs::Marker mk;

        mk.header.frame_id = "map";
        mk.header.stamp = ros::Time::now();
        mk.type = visualization_msgs::Marker::SPHERE_LIST;
        mk.action = visualization_msgs::Marker::ADD;
        mk.id = arc_bspline_traj_.markers.size();
        mk.scale.x = 0.2;
        mk.scale.y = 0.2;
        mk.scale.z = 0.2;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
        mk.color.a = 0.5;
        mk.color.r = 1.000;
        mk.color.g = 0.890;
        mk.color.b = 0.000;

        for (auto &point: waypoints)
        {
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            mk.points.push_back(p);
        }
        arc_bspline_traj_.markers.push_back(mk);

        // 发布新的标记
        arc_bspline_pub_->publish(arc_bspline_traj_);
    }


    void MotionManager::pubAstarTraj(const std::vector<Eigen::Vector3d> &waypoints)
    {
        // 删除上一次的标记
        deleteMarkerArray(astar_traj_);
        astar_pub_->publish(astar_traj_);
        astar_traj_.markers.clear();

        geometry_msgs::Point p;
        visualization_msgs::Marker mk;

        // 线段部分
        mk.header.frame_id = "map";
        mk.header.stamp = ros::Time::now();
        mk.type = visualization_msgs::Marker::LINE_STRIP;
        mk.action = visualization_msgs::Marker::ADD;
        mk.id = astar_traj_.markers.size();
        mk.scale.x = 0.1;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
        mk.color.a = 0.7;
        mk.color.r = 0.000;
        mk.color.g = 0.000;
        mk.color.b = 1.000;

        for (auto &point: waypoints)
        {
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            mk.points.push_back(p);
        }
        astar_traj_.markers.push_back(mk);

        // 端点部分
        mk = visualization_msgs::Marker();
        mk.header.frame_id = "map";
        mk.header.stamp = ros::Time::now();
        mk.type = visualization_msgs::Marker::SPHERE_LIST;
        mk.action = visualization_msgs::Marker::ADD;
        mk.id = astar_traj_.markers.size();
        mk.scale.x = 0.2;
        mk.scale.y = 0.2;
        mk.scale.z = 0.2;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;
        mk.color.a = 0.7;
        mk.color.r = 0.000;
        mk.color.g = 0.000;
        mk.color.b = 1.000;

        for (auto &point: waypoints)
        {
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            mk.points.push_back(p);
        }
        astar_traj_.markers.push_back(mk);

        // 发布新的标记
        astar_pub_->publish(astar_traj_);
    }


    void MotionManager::visTimerCallback(const ros::TimerEvent& event)
    {
        if (global_waypoints_.size() > 0)           pubAstarTraj(global_waypoints_);
        if (time_bspline_waypoints_.size() > 0)     pubReferTimeBspline(time_bspline_waypoints_);
        if (arc_bspline_waypoints_.size() > 0)      pubReferArcBspline(arc_bspline_waypoints_);
    }


    void MotionManager::resetMotion()
    {
        arc_ts_ = 0.0;
        arc_length_ = 0.0;
        bspline_mutex_.lock();
        bspline_exec_ = nullptr;
        bspline_mutex_.unlock();

        last_u_ << 0.0, 0.0, 0.0, haver_ratio_;

        for (int i = 0; i < MPCC::n_step_; i++)
        {
            u_predict_.block(i, 0, 1, 4) = last_u_.transpose();
        }

        for (int i = 0; i < 11; i++)
        {
            t_index_[i] = 0.1 * i;
        }
    }


    void MotionManager::visCleaner()
    {
        // 清理残留
        if (astar_traj_.markers.size() != 0)
        {
            // 发布删除命令
            deleteMarkerArray(astar_traj_);
            astar_pub_->publish(astar_traj_);
            astar_traj_.markers.clear();
        }

        if (time_bspline_traj_.markers.size() != 0)
        {
            deleteMarkerArray(time_bspline_traj_);
            time_bspline_pub_->publish(time_bspline_traj_);
            time_bspline_traj_.markers.clear();
            
        }

        if (arc_bspline_traj_.markers.size() != 0)
        {
            deleteMarkerArray(arc_bspline_traj_);
            arc_bspline_pub_->publish(arc_bspline_traj_);
            arc_bspline_traj_.markers.clear();
        }

        if (predict_state_.markers.size() != 0)
        {
            deleteMarkerArray(predict_state_);
            predict_state_pub_->publish(predict_state_);   
            predict_state_.markers.clear();         
        }
    }
}