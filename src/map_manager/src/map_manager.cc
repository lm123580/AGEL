#include "camera_util.h"
#include "map_manager.h"
#include "sdf_map.h"
#include "obstacle_util.h"
#include "frontier.h"
#include "kalman_filter.h"

namespace AGEL
{
    MapManager::MapManager() = default;
    MapManager::~MapManager() = default;

    void MapManager::init(ros::NodeHandle &nh)
    {
        start_time_ = ros::Time::now();

        nh.param("map_manager/frame_id", frame_id_, std::string("map"));
        nh.param("map_manager/map_ground_filter_hight", map_ground_filter_hight_, 0.4);

        camera_.reset(new CameraUtil(nh));
        
        map_.reset(new SDFMap);
        map_->initMap(nh);
        
        obstacle_.reset(new ObstacleUtil());
        obstacle_->init(map_);

        frontier_.reset(new Frontier(nh, map_));

        vis_timer_ = nh.createTimer(ros::Duration(0.30), &MapManager::visCallback, this);
        // dynamic_timer_ = nh.createTimer(ros::Duration(0.30), &MapManager::dynamicCallback, this);
        
        map_global_pub_ = std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_all", 1)); 
        map_local_pub_ = std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local", 1));
        
        map_global_inflate_pub_ = std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_global_inflate", 1));
        map_local_inflate_pub_ = std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local_inflate", 1));
        
        unknown_pub_ = std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 1));
        update_range_pub_ = std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 1));
        
        dynamic_pub_ = std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("/Object/dynamic", 1));

        frontiers_pub_ = std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("/explore/frontiers", 1));

        
        debug_1_pub_ = std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/debug_1", 1));
        debug_2_pub_ = std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/debug_2", 1));

        depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh, "/camera/depth", 1);
        pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::PoseStamped>>(nh, "/iris/pose", 1);
        sync_image_pose_ = std::make_shared<message_filters::Synchronizer<MapManager::SyncPolicyImagePose>>(MapManager::SyncPolicyImagePose(2), *depth_sub_, *pose_sub_);
        sync_image_pose_->registerCallback(boost::bind(&MapManager::depthPoseCallback, this, _1, _2));
    
        // debug_time_1_ = ros::Time::now();
    }


    void MapManager::depthPoseCallback(const sensor_msgs::ImageConstPtr &msgDepth, const geometry_msgs::PoseStampedConstPtr &msgOdom)
    {
        ros::Time time = ros::Time::now();

        if ((time - start_time_).toSec() < 10.0)
        {
            // std::cout << "Time is too short, waiting for the system to stabilize" << std::endl;
            return ;
        }
        
        // std::cout << "DepthPoseCallback time is:" << (time - debug_time_1_).toSec() * 1000 << "ms" << std::endl;
        // debug_time_1_ = time;
        cv_bridge::CvImageConstPtr cv_ptrDepth = cv_bridge::toCvShare(msgDepth);
        cv::Mat depthImg = cv_ptrDepth->image;

        Eigen::Vector3d t(msgOdom->pose.position.x, msgOdom->pose.position.y, msgOdom->pose.position.z);
        Eigen::Quaterniond Q(msgOdom->pose.orientation.w, msgOdom->pose.orientation.x, msgOdom->pose.orientation.y, msgOdom->pose.orientation.z);
        Eigen::Matrix4d camera_pos = Eigen::Matrix4d::Identity();
        camera_pos.block<3, 3>(0, 0) = Q.toRotationMatrix();
        camera_pos.block<3, 1>(0, 3) = t;

        // 更新相机位姿
        camera_->updatePose(camera_pos);
        pcl::PointCloud<pcl::PointXYZ>::Ptr rayCloud(new pcl::PointCloud<pcl::PointXYZ>), obCloud(new pcl::PointCloud<pcl::PointXYZ>);
        // ros::Time t1 = ros::Time::now();

        // 得到点云
        depthPose2Cloud(depthImg, rayCloud, obCloud);
        // ros::Time t2 = ros::Time::now();

        // 分类点云
        obstacle_->update(obCloud, camera_, time);
        ros::Time t3 = ros::Time::now();

        // 更新空闲点云
        map_->RayUpdateFree(*rayCloud, t);
        // ros::Time t4 = ros::Time::now();

        // 更新静态点云
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector;
        obstacle_->getStaticPCL(cloud_vector);

        for (auto &cloud_v: cloud_vector)
        {
            map_->updateOccupied(*cloud_v);
        }

        // 更新动态物体
        updateDynamic(obstacle_);
        
        // 膨胀静态地图
        map_->inflateLocalMap();

        // 更新探索区域
        map_->updateExploreMap();
        ros::Time t5 = ros::Time::now();

        // 更新前沿
        frontier_->updateFrontiers();
        
        map_dyn_updat_time_info_.num++;
        map_dyn_updat_time_info_.total_time += ((t5 - t3).toSec() * 1000.0);

        info_num_++;
        if (info_num_ % 150 == 0)
        {
            std::cout << "地图更新及动态物体维护时间：" << (map_dyn_updat_time_info_.total_time / map_dyn_updat_time_info_.num) << std::endl;
            info_num_ = 0;
        }

        // 发布动态物体
        pubDynamic();

        // 发布前沿
        pub_frontier_num_++;
        if (pub_frontier_num_ > 10)
        {
            pubFrontiers();
            pub_frontier_num_ = 0;
        }

        // ros::Time t5 = ros::Time::now();

        // std::cout << "Depth to cloud time is " << (t2 - t1).toSec() * 1000 << "ms" << std::endl;
        // std::cout << "RayUpdateFree time is " << (t3 - t2).toSec() * 1000 << "ms" << std::endl;
        // std::cout << "Obstacle classifay time is " << (t4 - t3).toSec() * 1000 << "ms" << std::endl;
        // std::cout << "Update static infomation time is " << (t5 - t4).toSec() * 1000 << "ms" << std::endl;
        // std::cout << "Map Total time is " << (t5 - time).toSec() * 1000 << "ms" << std::endl;
  
        // // 发布debug点云
        // obCloud->width = obCloud->points.size();
        // obCloud->height = 1;
        // obCloud->header.frame_id = frame_id_;
        // sensor_msgs::PointCloud2 cloud_msg;
        // pcl::toROSMsg(*obCloud, cloud_msg);
        // debug_1_pub_->publish(cloud_msg);
    }


    void MapManager::updateDynamic(std::shared_ptr<ObstacleUtil> &ob)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector;
        std::vector<Eigen::Vector3d> poses, max_sizes;
        std::vector<std::shared_ptr<KalmanFilter>> kf_vector;

        ob->getDynamicOb(cloud_vector, poses, max_sizes, kf_vector);

        dynamic_objects_ = std::vector<DynamicObject>();
        for (int i = 0; i < (int)cloud_vector.size(); i++)
        {
            DynamicObject obj;
            obj.pos = poses[i];
            obj.max_size = max_sizes[i];
            obj.kf_ptr = kf_vector[i];
            obj.cloud_ptr = cloud_vector[i];

            dynamic_objects_.push_back(obj);
        }
    }


    void MapManager::visCallback(const ros::TimerEvent &event)
    {
        // ros::Time t1 = ros::Time::now();
        publishMapGlobal();
        // publishMapLocal();
        publishUnknown();
        // publishESDF();
        // publishUpdateRange();
        // ros::Time t2 = ros::Time::now();
        // ROS_INFO("publish time : %.5f", (t2 - t1).toSec());
    }


    void MapManager::pubDynamic()
    {
        // 删除上一时刻动态物体
        deleteDynamicObjectMarkers();
        dynamic_pub_->publish(mk_dynamics_);

        // 更新为现在的动态物体
        updateDynamicObjectMarkers();
        dynamic_pub_->publish(mk_dynamics_);
    }


    void MapManager::dynamicCallback(const ros::TimerEvent &event)
    {
        // 删除上一时刻动态物体
        deleteDynamicObjectMarkers();
        dynamic_pub_->publish(mk_dynamics_);

        // 更新为现在的动态物体
        updateDynamicObjectMarkers();
        dynamic_pub_->publish(mk_dynamics_);
    }

    void MapManager::addDynamicObjectMarker(DynamicObject &obj)
    {
        // Ellipsoid
        visualization_msgs::Marker mk_ellipsoid;

        mk_ellipsoid.header.frame_id = frame_id_;
        mk_ellipsoid.header.stamp = ros::Time::now();
        mk_ellipsoid.type = visualization_msgs::Marker::SPHERE;
        mk_ellipsoid.action = visualization_msgs::Marker::ADD;
        mk_ellipsoid.id = mk_dynamics_.markers.size();
        mk_ellipsoid.pose.position.x = obj.pos[0];
        mk_ellipsoid.pose.position.y = obj.pos[1];
        mk_ellipsoid.pose.position.z = obj.pos[2];
        mk_ellipsoid.pose.orientation.w = 1.0;
        mk_ellipsoid.scale.x = obj.max_size[0];
        mk_ellipsoid.scale.y = obj.max_size[1];
        mk_ellipsoid.scale.z = obj.max_size[2];
        mk_ellipsoid.color.a = 1.0;
        mk_ellipsoid.color.r = 1.0;
        mk_ellipsoid.color.g = 0.0;
        mk_ellipsoid.color.b = 0.0;
        
        mk_dynamics_.markers.push_back(mk_ellipsoid);

        // Cylinder
        visualization_msgs::Marker mk_cy;
        mk_cy.header.frame_id = frame_id_;
        mk_cy.header.stamp = ros::Time::now();
        mk_cy.type = visualization_msgs::Marker::CYLINDER;
        mk_cy.action = visualization_msgs::Marker::ADD;
        mk_cy.color.a = 0.3;
        mk_cy.color.r = 0.0;
        mk_cy.color.g = 1.0;
        mk_cy.color.b = 0.0;

        mk_cy.pose.orientation.w = 1.0;
        mk_cy.scale.x = 1.5 * obj.max_size[0];
        mk_cy.scale.y = 1.5 * obj.max_size[1];
        mk_cy.scale.z = 0.5 * obj.max_size[2];

        double time_pre;
        Eigen::Matrix<double, 6, 1> pre_state;
        for (int i = 0; i < 10; i++)
        {   
            time_pre = i * 0.05;
            pre_state = obj.kf_ptr->predict(time_pre);

            mk_cy.id = mk_dynamics_.markers.size();
            mk_cy.pose.position.x = pre_state[0];
            mk_cy.pose.position.y = pre_state[1];
            mk_cy.pose.position.z = 0.75 * pre_state[2];
            mk_cy.scale.x = mk_cy.scale.x * 1.025;
            mk_cy.scale.y = mk_cy.scale.y * 1.025;

            mk_dynamics_.markers.push_back(mk_cy);
        }
    }


    void MapManager::deleteDynamicObjectMarkers()
    {
        for (auto &mk: mk_dynamics_.markers)
        {
            mk.action = visualization_msgs::Marker::DELETE;
        }
    }


    void MapManager::updateDynamicObjectMarkers()
    {
        visualization_msgs::MarkerArray new_mk_dynamics;
        mk_dynamics_ = new_mk_dynamics;
        
        int obj_size = dynamic_objects_.size();

        for (int i = 0; i < obj_size; i++)
        {
            addDynamicObjectMarker(dynamic_objects_[i]);
        }
    }


    void MapManager::getDynamicInfo(std::vector<Eigen::Matrix<double, 9, 1>> &ob_dynamics)
    {
        // int dynamic_size;
        std::vector<DynamicObject> dynamic_objects;

        dynamic_objects = dynamic_objects_;

        // dynamic_size = dynamic_objects.size();

        ob_dynamics.clear();
        for (auto &ob: dynamic_objects)
        {
            Eigen::Matrix<double, 9, 1> tmp_ob;

            tmp_ob.block<3, 1>(0, 0) = ob.max_size;
            tmp_ob.block<3, 1>(3, 0) = ob.pos;
            tmp_ob.block<3, 1>(6, 0) = ob.kf_ptr->getVel();

            ob_dynamics.emplace_back(tmp_ob);
        }
    }


    void MapManager::depthPose2Cloud(const cv::Mat &depthImg, pcl::PointCloud<pcl::PointXYZ>::Ptr &rayCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &obCloud)
    {
        pcl::PointXYZ point;
        Eigen::Vector3d point_pos;

        Eigen::Matrix3d rotation_matrix = camera_->Twc_.block<3, 3>(0, 0);
        Eigen::Vector3d translation_vector = camera_->Twc_.block<3, 1>(0, 3);

        
        int skip_pixel = 3;

        // 基本参数获取
        int img_rows = camera_->img_rows_;
        int img_cols = camera_->img_cols_;

        double cx = camera_->cx_;
        double cy = camera_->cy_;
        double fx = camera_->fx_;
        double fy = camera_->fy_;
        double depth_filter_maxdist = camera_->max_dist_;
        double depth_filter_mindist = camera_->min_dist_;

        double d;
        Eigen::Vector3d dir;
        // ros::Time t1 = ros::Time::now();
        for (int u = 0; u < img_rows; u = u + skip_pixel)
        {
            for (int v = 0; v < img_cols; v = v + skip_pixel)
            {
                d = (double)depthImg.at<float>(u, v);

                // if (d < depth_filter_mindist || isnanf(d))
                // {
                //     continue;
                // }

                // if (d > depth_filter_maxdist)
                // {
                //     d = depth_filter_maxdist;
                // }

                if (d < depth_filter_mindist)
                {
                    continue;
                }

                if (d > depth_filter_maxdist || isnanf(d))
                {
                    d = depth_filter_maxdist;
                }

                point_pos[0] = (v - cx) * d / fx;
                point_pos[1] = (u - cy) * d / fy;
                point_pos[2] = d;

                point_pos = rotation_matrix * point_pos + translation_vector;
                
                if (point_pos[2] > depth_filter_mindist)
                {
                    point.x = point_pos[0];
                    point.y = point_pos[1];
                    point.z = point_pos[2];

                    rayCloud->push_back(point);

                    if(d < depth_filter_maxdist && point_pos[2] > map_ground_filter_hight_ && point_pos[2] < 2.0)
                    {
                        obCloud->push_back(point);
                    }
                }
            }
        }
        // ros::Time t2 = ros::Time::now();
        // std::cout << "---> Obstacle number is " << obCloud->size() << std::endl;
        // std::cout << "---> get cloud time is " << (t2 - t1).toSec() * 1000 << "ms" << std::endl;
        // std::cout << "count_nan is :" << count_nan << std::endl;
        
        pcl::VoxelGrid<pcl::PointXYZ> sor;

        sor.setInputCloud(obCloud);
        sor.setLeafSize(0.1f, 0.1f, 0.1f);
        sor.filter(*obCloud);

        // sor.setInputCloud(rayCloud);
        // sor.setLeafSize(0.02f, 0.02f, 0.02f);
        // sor.filter(*rayCloud);

        // ros::Time t3 = ros::Time::now();
        // std::cout << "---> Filter obstacle number is " << obCloud->size() << std::endl;
        // std::cout << "---> voxel grid time is " << (t3 - t2).toSec() * 1000 << "ms" << std::endl;
    }


    void MapManager::publishMapGlobal() 
    {
        pcl::PointXYZ point;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);


        for (int x = map_->mp_->box_min_(0); x < map_->mp_->box_max_(0); x++)
        {
            for (int y = map_->mp_->box_min_(1); y < map_->mp_->box_max_(1); y++)
            {
                for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); z++)
                {
                    if (map_->getOccupancy(Eigen::Vector3i(x, y, z)) == map_->OCCUPIED)
                    {
                        Eigen::Vector3d pos;
                        map_->indexToPos(Eigen::Vector3i(x, y, z), pos);

                        // if (pos(2) > visualization_truncate_low_ && pos(2) < visualization_truncate_height_)
                        // {
                            point.x = pos(0);
                            point.y = pos(1);
                            point.z = pos(2);

                            cloud->push_back(point);
                        // }
                    }

                    if (map_->md_->occupancy_buffer_inflate_[map_->toAddress(x, y, z)] == true)
                    {
                        Eigen::Vector3d pos;
                        map_->indexToPos(Eigen::Vector3i(x, y, z), pos);

                        // if (pos(2) > visualization_truncate_low_ && pos(2) < visualization_truncate_height_)
                        // {
                            point.x = pos(0);
                            point.y = pos(1);
                            point.z = pos(2);

                            cloud2->push_back(point);
                        // }
                    }
                }
            }
        }

        sensor_msgs::PointCloud2 cloud_msg;

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->header.frame_id = frame_id_;        
        cloud2->width = cloud2->points.size();
        cloud2->height = 1;
        cloud2->header.frame_id = frame_id_; 

        pcl::toROSMsg(*cloud, cloud_msg);        
        map_global_pub_->publish(cloud_msg);

        pcl::toROSMsg(*cloud2, cloud_msg);  
        map_global_inflate_pub_->publish(cloud_msg);   
    }


    void MapManager::publishMapLocal() 
    {
        pcl::PointXYZ point;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        
        Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
        Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
        map_->boundIndex(min_cut);
        map_->boundIndex(max_cut);

        for (int x = min_cut(0); x < max_cut(0); x++)
        {
            for (int y = min_cut(1); y < max_cut(1); y++)
            {
                for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); z++)
                {
                    if (map_->getOccupancy(Eigen::Vector3i(x, y, z)) == map_->OCCUPIED)
                    {
                        Eigen::Vector3d pos;
                        map_->indexToPos(Eigen::Vector3i(x, y, z), pos);

                        // if (pos(2) > visualization_truncate_low_ && pos(2) < visualization_truncate_height_)
                        // {
                            point.x = pos(0);
                            point.y = pos(1);
                            point.z = pos(2);

                            cloud->push_back(point);
                        // }
                    }

                    if (map_->md_->occupancy_buffer_inflate_[map_->toAddress(x, y, z)] == true)
                    {
                        Eigen::Vector3d pos;
                        map_->indexToPos(Eigen::Vector3i(x, y, z), pos);

                        // if (pos(2) > visualization_truncate_low_ && pos(2) < visualization_truncate_height_)
                        // {
                            point.x = pos(0);
                            point.y = pos(1);
                            point.z = pos(2);

                            cloud2->push_back(point);
                        // }
                    }
                }
            }
        }

        sensor_msgs::PointCloud2 cloud_msg;

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->header.frame_id = frame_id_;        
        cloud2->width = cloud2->points.size();
        cloud2->height = 1;
        cloud2->header.frame_id = frame_id_; 

        pcl::toROSMsg(*cloud, cloud_msg);        
        map_local_pub_->publish(cloud_msg);

        pcl::toROSMsg(*cloud2, cloud_msg);  
        map_local_inflate_pub_->publish(cloud_msg);   
    }


    void MapManager::publishUnknown() {
        pcl::PointXYZ point;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
        Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
        map_->boundIndex(max_cut);
        map_->boundIndex(min_cut);

        for (int x = map_->mp_->box_min_(0); x < map_->mp_->box_max_(0); x++)
        {
            for (int y = map_->mp_->box_min_(1); y < map_->mp_->box_max_(1); y++)
            {
                if (map_->get2DState(Eigen::Vector2i(x, y)) != map_->UNKNOWN)
                {
                    Eigen::Vector2d pos;
                    map_->indexToPos(Eigen::Vector2i(x, y), pos);

                    point.x = pos(0);
                    point.y = pos(1);
                    point.z = map_ground_filter_hight_;

                    cloud->push_back(point);
                }
            }
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->header.frame_id = frame_id_;

        sensor_msgs::PointCloud2 cloud_msg;
        
        pcl::toROSMsg(*cloud, cloud_msg);
        
        unknown_pub_->publish(cloud_msg);            
    }


    void MapManager::publishUpdateRange() 
    {
        Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
        visualization_msgs::Marker mk;
        map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
        map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

        cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
        cube_scale = esdf_max_pos - esdf_min_pos;
        
        mk.header.frame_id = frame_id_;
        mk.header.stamp = ros::Time::now();
        mk.type = visualization_msgs::Marker::CUBE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.id = 0;
        
        mk.pose.position.x = cube_pos(0);
        mk.pose.position.y = cube_pos(1);
        mk.pose.position.z = cube_pos(2);

        mk.pose.orientation.w = 1.0;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        
        mk.scale.x = cube_scale(0);
        mk.scale.y = cube_scale(1);
        mk.scale.z = cube_scale(2);
        
        mk.color.a = 0.3;
        mk.color.r = 1.0;
        mk.color.g = 0.0;
        mk.color.b = 0.0;
        
        update_range_pub_->publish(mk);
    }


    void MapManager::pubFrontiers()
    {
        // 删除上一次边界
        deleteMarkerArray(mk_frontiers_);
        frontiers_pub_->publish(mk_frontiers_);

        mk_frontiers_ = visualization_msgs::MarkerArray();

        double scale_x = map_->getResolution();
        double scale_y = map_->getResolution();
        double scale_z = map_->getMapSizeZ() - map_->getGroundHeight();
        double center_z = (map_->getMapSizeZ() + map_->getGroundHeight()) * 0.5;

        Eigen::Vector4d color;
        std::vector<FIS> &fises = frontier_->frontiers_;
        int fis_size = fises.size();

        for (int i = 0; i < fis_size; i++)
        {
            visualization_msgs::Marker mk;
            mk.header.frame_id = "map";
            mk.header.stamp = ros::Time::now();
            mk.type = visualization_msgs::Marker::CUBE_LIST;
            mk.action = visualization_msgs::Marker::ADD;
            mk.id = i;
            mk.scale.x = scale_x;
            mk.scale.y = scale_y;
            mk.scale.z = scale_z;
            mk.pose.orientation.w = 1.0;
            mk.pose.orientation.x = 0.0;
            mk.pose.orientation.y = 0.0;
            mk.pose.orientation.z = 0.0;

            color = getColor(i, 0.8);
            mk.color.r = color[0];
            mk.color.g = color[1];
            mk.color.b = color[2];
            mk.color.a = color[3];
            
            geometry_msgs::Point pt;
            std::vector<Eigen::Vector2d> cells = fises[i].cells;
            for (const auto &cell: cells)
            {
                pt.x = cell[0];
                pt.y = cell[1];
                pt.z = center_z;
                mk.points.push_back(pt);
            }

            mk_frontiers_.markers.push_back(mk);
        }

        // 发布新的标记
        frontiers_pub_->publish(mk_frontiers_);
    }


    void MapManager::deleteMarkerArray(visualization_msgs::MarkerArray &mk_array)
    {
        int mk_size = mk_array.markers.size();
        for (int i = 0; i < mk_size; i++)
        {
            mk_array.markers[i].action = visualization_msgs::Marker::DELETE;
        }
    }


    Eigen::Vector4d MapManager::getColor(const int& h, double alpha) 
    {
        double h1 = (double)(h % 6) / 6.0 + 1e-3;

        Eigen::Vector4d color1, color2;
        if (h1 >= -1e-4 && h1 < 1.0 / 6) 
        {
            color1 = Eigen::Vector4d(1, 0, 0, 1);
            color2 = Eigen::Vector4d(1, 0, 1, 1);
        } 
        else if (h1 >= 1.0 / 6 && h1 < 2.0 / 6) 
        {
            color1 = Eigen::Vector4d(1, 0, 1, 1);
            color2 = Eigen::Vector4d(0, 0, 1, 1);
        } 
        else if (h1 >= 2.0 / 6 && h1 < 3.0 / 6) 
        {
            color1 = Eigen::Vector4d(0, 0, 1, 1);
            color2 = Eigen::Vector4d(0, 1, 1, 1);
        } 
        else if (h1 >= 3.0 / 6 && h1 < 4.0 / 6) 
        {
            color1 = Eigen::Vector4d(0, 1, 1, 1);
            color2 = Eigen::Vector4d(0, 1, 0, 1);
        } 
        else if (h1 >= 4.0 / 6 && h1 < 5.0 / 6) 
        {
            color1 = Eigen::Vector4d(0, 1, 0, 1);
            color2 = Eigen::Vector4d(1, 1, 0, 1);
        } 
        else if (h1 >= 5.0 / 6 && h1 <= 1.0 + 1e-4) 
        {
            color1 = Eigen::Vector4d(1, 1, 0, 1);
            color2 = Eigen::Vector4d(1, 0, 0, 1);
        }

        Eigen::Vector4d fcolor = 0.5 * color1 + 0.5 * color2;
        fcolor(3) = alpha;

        return fcolor;
    }
}