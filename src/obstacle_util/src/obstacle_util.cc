#include "sdf_map.h"
#include "camera_util.h"
#include "kalman_filter.h"
#include "DBSCANKdtreeCluster.h"

#include "obstacle_util.h"

namespace AGEL
{
    ObstacleUtil::ObstacleUtil() = default;
    ObstacleUtil::~ObstacleUtil() = default;

    void ObstacleUtil::init(std::shared_ptr<SDFMap> &map)
    {
        map_ptr_ = map;
        dbscan_kdtree_ptr_.reset(new DBSCANKdtreeCluster);
        Threshold_diff_ = 0.5;
        feature_weighting_matrix_ = Eigen::MatrixXd::Identity(6, 6);
        feature_weighting_matrix_(3, 3) = 0.15;
        feature_weighting_matrix_(4, 4) = 0.15;
        feature_weighting_matrix_(5, 5) = 0.15;
    }


    void ObstacleUtil::update(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::shared_ptr<CameraUtil> &camera, ros::Time &time)
    {
        std::vector<std::shared_ptr<CloudCluster>> cluster_vector; 
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector;

        ros::Time t1, t2, t3, t4;

        t1 = ros::Time::now();

        dbscan_kdtree_ptr_->run(cloud, cloud_vector);                       // 获取点云集合

        cloud2cluster(cloud_vector, cluster_vector, camera, time);          // 将点云集合转化为簇集合
        
        t2 = ros::Time::now();

        int cswv_size = cswv_.size();
        int cluster_size = cluster_vector.size();

        std::vector<int> ob2c_indices(cswv_size, -1);                       // 物体与簇匹配索引
        std::vector<bool> cluster_match_flags(cluster_size, false);         // 簇的匹配标志
        
        // 将障碍物与新簇匹配，并得到匹配索引
        matchObandC(cluster_vector, ob2c_indices, cluster_match_flags);
        
        // 根据索引将簇添加进对应物体的时间窗口，并滑动时间窗口
        int temp_index;
        for (int i = 0; i < cswv_size; i++)
        {
            temp_index = ob2c_indices[i];

            if (temp_index >= 0)
            {
                Eigen::Vector3d temp_xyz = cluster_vector[temp_index]->max_point - cluster_vector[temp_index]->min_point;
                cswv_[i].cluster_queue.push_back(cluster_vector[temp_index]);
                cswv_[i].max_x = std::max(temp_xyz[0], cswv_[i].max_x);
                cswv_[i].max_y = std::max(temp_xyz[1], cswv_[i].max_y);
                cswv_[i].max_z = std::max(temp_xyz[2], cswv_[i].max_z);

                if (cswv_[i].cluster_queue.size() > 6)                          // 时间窗口的容量限制暂定为6
                {
                    cswv_[i].cluster_queue.pop_front();
                }
            }
        }

        // 删除太久未匹配上的物体
        for (int i = 0; i < cswv_size; i++)
        {
            if ((time - cswv_[i].cluster_queue.back()->time).toSec() > 0.25)     // 阈值为0.25s
            {
                cswv_.erase(cswv_.begin() + i);                                 // 注意，删除或增加都会改变窗口的大小
                cswv_size--;
                i--;
            }
        }

        // 为未匹配的簇创建一个障碍物时间窗口
        for (int i = 0; i < cluster_size; i++)
        {
            if (!cluster_match_flags[i])                                        // 如果簇未被匹配
            {
                CSW temp_csw;

                temp_csw.cluster_queue.push_back(cluster_vector[i]);
                temp_csw.kf_ptr.reset(new KalmanFilter);
                temp_csw.max_x = cluster_vector[i]->max_point[0] - cluster_vector[i]->min_point[0];
                temp_csw.max_y = cluster_vector[i]->max_point[1] - cluster_vector[i]->min_point[1];
                temp_csw.max_z = cluster_vector[i]->max_point[2] - cluster_vector[i]->min_point[2];

                cswv_.push_back(temp_csw);
            }
        }

        t3 = ros::Time::now();

        bool is_df = true;
        // 遍历时间窗口，更新障碍物状态（如果障碍物运动，则更新相对应的卡尔曼状态）
        for (auto &csw: cswv_)
        {
            if (csw.cluster_queue.back()->time == time)                                         // 确保时间窗口是已经更新的
            {
                int csw_size = csw.cluster_queue.size();

                if (csw_size < 6)
                {
                    getMapPost(csw.cluster_queue.back(), csw.state);                            // 时间间隔小，利用贝叶斯推断更新后验
                    is_df = false;
                }
                else
                {   
                    getFDPost(csw.cluster_queue.back(), csw.cluster_queue.front(), csw.state);  // 时间间隔大，利用帧差法更新后验
                    
                    // // 使用贝叶斯推断再确认一遍
                    // if (csw.state == DYNAMIC)
                    // {
                    //     STATE tmp_state;
                    //     int tmp_count = 0;
                    //     for (auto &tmp_cluster: csw.cluster_queue)
                    //     {
                    //         getMapPost(tmp_cluster, tmp_state);
                    //         if (tmp_state == DYNAMIC)
                    //         {
                    //             tmp_count++;
                    //         }
                    //     }

                    //     if (tmp_count <= 4)
                    //     {
                    //         csw.state = STATIC;
                    //     }
                    // }
                }

                if (csw.state == DYNAMIC)
                {
                    // Eigen::Vector3d &measurement = csw.cluster_queue.back()->quality_point;
                    csw.kf_ptr->update(csw.cluster_queue.back()->quality_point, time);          // 更新卡尔曼滤波器
                }                
            }
        }

        t4 = ros::Time::now();

        updateTimeInfo(cluster_time_info_, (t2 - t1));
        updateTimeInfo(dada_ass_time_info_, (t3 - t2));
        if (is_df)
        {
            updateTimeInfo(vote_time_info_, (t4 - t3));
        }
        else
        {
            updateTimeInfo(beyes_time_info_, (t4 - t3));
        }
        updateTimeInfo(classifay_time_info_, (t4 - t3));
        print_num_++;
        if (print_num_ % 150 == 0)
        {
            std::cout << "\n\n-----------------------" << std::endl;
            std::cout << "密度聚类时间："<< (cluster_time_info_.total_time / cluster_time_info_.num )<<std::endl;
            std::cout << "数据关联时间："<< (dada_ass_time_info_.total_time / dada_ass_time_info_.num )<<std::endl;
            std::cout << "速度投票时间："<< (vote_time_info_.total_time / vote_time_info_.num )<<std::endl;
            std::cout << "贝叶斯推断时间："<< (beyes_time_info_.total_time / beyes_time_info_.num )<<std::endl;
            std::cout << "分类时间："<< (classifay_time_info_.total_time / classifay_time_info_.num )<<std::endl;            
            print_num_ = 0;
        }
    }

    void ObstacleUtil::updateTimeInfo(TimeInfo &info, ros::Duration t)
    {
        info.num++;
        info.total_time += (t.toSec() * 1000.0);
    }


    void ObstacleUtil::cloud2cluster(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_vector,
                                     std::vector<std::shared_ptr<CloudCluster>> &cluster_vector,
                                     std::shared_ptr<CameraUtil> &camera, ros::Time &time)
    {
        int cloud_size = cloud_vector.size();
        
        cluster_vector.resize(cloud_size);
        
        for (int i = 0; i < cloud_size; i++)
        {
            Eigen::Vector3d quality_point, min_point, max_point;
            pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_temp(new pcl::search::KdTree<pcl::PointXYZ>);

            quality_point << 0.0, 0.0, 0.0;                                                 // 获取质心和AABB框
            min_point << std::numeric_limits<double>::max(),                
                         std::numeric_limits<double>::max(),
                         std::numeric_limits<double>::max();
            max_point << -std::numeric_limits<double>::max(),
                         -std::numeric_limits<double>::max(),
                         -std::numeric_limits<double>::max();

            for (auto &point: *cloud_vector[i])
            {
                quality_point[0] = quality_point[0] + point.x;
                quality_point[1] = quality_point[1] + point.y;
                quality_point[2] = quality_point[2] + point.z;

                if(point.x < min_point[0]) min_point[0] = point.x;
                if(point.y < min_point[1]) min_point[1] = point.y;
                if(point.z < min_point[2]) min_point[2] = point.z;
                if(point.x > max_point[0]) max_point[0] = point.x;
                if(point.y > max_point[1]) max_point[1] = point.y;
                if(point.z > max_point[2]) max_point[2] = point.z;
            }

            quality_point = quality_point / cloud_vector[i]->size();

            kdtree_temp->setInputCloud(cloud_vector[i]);                                    // 获取KD树
            
            cluster_vector[i].reset(new CloudCluster);

            cluster_vector[i]->time = time;                                                 // 时间戳
            
            cluster_vector[i]->camera_util_ptr = std::make_shared<CameraUtil>(*camera);     // 相机参数需要拷贝
            
            cluster_vector[i]->quality_point = quality_point;                               // 质心

            cluster_vector[i]->min_point = min_point;                                       // AABB

            cluster_vector[i]->max_point = max_point;

            cluster_vector[i]->cloud_ptr = cloud_vector[i];                                 // 点云

            cluster_vector[i]->kdtree_ptr = kdtree_temp;                                    // KDTree
        }
    }


    void ObstacleUtil::matchObandC(std::vector<std::shared_ptr<CloudCluster>> &cluster_vector,
                                   std::vector<int> &ob2c_indices, std::vector<bool> &cluster_match_flags)
    {
        int cswv_size = cswv_.size();
        int cluser_size = cluster_vector.size();

        ob2c_indices.resize(cswv_size, -1);
        cluster_match_flags.resize(cluser_size, false);

        for (int i = 0; i < cswv_size; i++)
        {
            double diff;
            double min_diff = std::numeric_limits<double>::max();   // 最小特征差距
            std::shared_ptr<CloudCluster> top_ptr;                  // 时间窗口顶部簇指针
            Eigen::VectorXd feature_top(6), feature_cluster(6);     // 时间窗口顶部簇特征和当前簇特征
            
            top_ptr = cswv_[i].cluster_queue.back();

            feature_top << top_ptr->quality_point[0], top_ptr->quality_point[1], top_ptr->quality_point[2],    // pos
                           top_ptr->max_point[0] - top_ptr->min_point[0],                                      // shape
                           top_ptr->max_point[1] - top_ptr->min_point[1],
                           top_ptr->max_point[2] - top_ptr->min_point[2];

            for (int j = 0; j < cluser_size; j++)
            {
                if (!cluster_match_flags[j])
                {
                    feature_cluster << cluster_vector[j]->quality_point[0], cluster_vector[j]->quality_point[1], cluster_vector[j]->quality_point[2],
                                       cluster_vector[j]->max_point[0] - cluster_vector[j]->min_point[0],
                                       cluster_vector[j]->max_point[1] - cluster_vector[j]->min_point[1],
                                       cluster_vector[j]->max_point[2] - cluster_vector[j]->min_point[2];

                    diff = (feature_weighting_matrix_ * (feature_top - feature_cluster)).norm();                // feature diff
                    
                    if (diff < min_diff)
                    {
                        min_diff = diff;
                        ob2c_indices[i] = j;
                    }
                }
            }

            if (min_diff < Threshold_diff_)
            {
                cluster_match_flags[ob2c_indices[i]] = true;
            }
            else
            {
                ob2c_indices[i] = -1;
            }
        }
    }


    void ObstacleUtil::getMapPost(std::shared_ptr<CloudCluster> &cluser, STATE &priori)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  

        cloud = cluser->cloud_ptr;                  // 观测点云
        int alpha = 1, beta = 1;                    // alpha静，beta动

        Eigen::Vector3d tmp_pos;
        for (auto &point: *cloud)
        {
            tmp_pos << point.x, point.y, point.z;

            if (map_ptr_->getInflateOccupancy(tmp_pos) /*|| map_ptr_->getOccupancy(tmp_pos) == map_ptr_->UNKNOWN*/)
            {
                alpha++;                            // 静止
            }
            else
            {
                beta++;                             // 运动
            }

            // if (map_ptr_->getInflateOccupancy(tmp_pos)) 
            // {
            //     alpha++;                        // 静止
            // }
            // else 
            // {
            //     beta++;                         // 运动
            // }
        }

        double max_probability = (alpha - 1) / (alpha + beta - 2);
        
        if (max_probability >= 0.5)
        {
            priori = STATIC;
        }
        else
        {
            priori = DYNAMIC;
        }
    }


    void ObstacleUtil::getFDPost(std::shared_ptr<CloudCluster> &cluser_1, std::shared_ptr<CloudCluster> &cluser_2, STATE &priori)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>), cloud_2(new pcl::PointCloud<pcl::PointXYZ>);

        filterFov(cluser_1->cloud_ptr, cloud_1, cluser_2->camera_util_ptr);
        filterFov(cluser_2->cloud_ptr, cloud_2, cluser_1->camera_util_ptr);

        double vote_score;
        int voter_nums, static_nums;
        if (cloud_1->size() > cloud_2->size())
        {
            static_nums = 0;
            voter_nums = cloud_2->size();

            int k = 1;
            double dis;
            std::vector<int> indices(k);
            std::vector<float> distances(k);

            pcl::search::KdTree<pcl::PointXYZ>::Ptr tmp_kdtree = cluser_1->kdtree_ptr;

            for (auto &point: *cloud_2)
            {
                tmp_kdtree->nearestKSearch(point, k, indices, distances);
                dis = std::sqrt(distances[0]);

                if (dis < 0.05)
                {
                    static_nums++;
                }
            }

            vote_score = (double)static_nums / (double)voter_nums;

            if (vote_score > 0.6)
            {
                priori = STATIC;
            }
            else
            {
                priori = DYNAMIC;
            }
        }
        else
        {
            static_nums = 0;
            voter_nums = cloud_1->size();

            int k = 1;
            double dis;
            std::vector<int> indices(k);
            std::vector<float> distances(k);

            pcl::search::KdTree<pcl::PointXYZ>::Ptr tmp_kdtree = cluser_2->kdtree_ptr;

            for (auto &point: *cloud_1)
            {
                tmp_kdtree->nearestKSearch(point, k, indices, distances);
                dis = std::sqrt(distances[0]);

                if (dis < 0.05)
                {
                    static_nums++;
                }
            }

            vote_score = (double)static_nums / (double)voter_nums;

            if (vote_score > 0.6)
            {
                priori = STATIC;
            }
            else
            {
                priori = DYNAMIC;
            }
        }
    }


    void ObstacleUtil::filterFov(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud, 
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud,
                                 std::shared_ptr<CameraUtil> &camera)
    {
        Eigen::Vector3d tmp_pos;
        for (auto &point: *in_cloud)
        {
            tmp_pos << point.x, point.y, point.z;
            if (camera->insideFOV(tmp_pos))
            {
                out_cloud->push_back(point);
            }
        }
    }


    // 得到静止障碍物
    void ObstacleUtil::getStaticPCL(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_vector)
    {
        cloud_vector.clear();

        for (auto &csw: cswv_)
        {
            if (csw.state == STATIC)
            {
                cloud_vector.push_back(csw.cluster_queue.back()->cloud_ptr);
            }
        }
    }


    // 得到动态障碍物
    void ObstacleUtil::getDynamicOb(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_vector,
                                    std::vector<Eigen::Vector3d> &poses,
                                    std::vector<Eigen::Vector3d> &max_sizes,
                                    std::vector<std::shared_ptr<KalmanFilter>> &kf_vector)
    {
        cloud_vector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>();
        poses = std::vector<Eigen::Vector3d>();
        max_sizes = std::vector<Eigen::Vector3d>();
        kf_vector = std::vector<std::shared_ptr<KalmanFilter>>();

        for (auto &csw: cswv_)
        {
            if (csw.state == DYNAMIC)
            {
                cloud_vector.push_back(csw.cluster_queue.back()->cloud_ptr);
                poses.push_back(csw.cluster_queue.back()->quality_point);
                max_sizes.push_back(Eigen::Vector3d(csw.max_x, csw.max_y, csw.max_z));
                kf_vector.push_back(csw.kf_ptr);
            }
        }
    }
}