#include "raycast.h"
#include "sdf_map.h"

namespace AGEL
{
    SDFMap::SDFMap() = default;
    SDFMap::~SDFMap() = default;

    void SDFMap::initMap(ros::NodeHandle &nh)
    {
        mp_.reset(new MapParam);
        md_.reset(new MapData);

        double map_hight_min;
        double x_size, y_size, z_size;
        
        nh.param("sdf_map/resolution", mp_->resolution_, 0.1);
        nh.param("sdf_map/map_size_x", x_size, 40.0);
        nh.param("sdf_map/map_size_y", y_size, 40.0);
        nh.param("sdf_map/map_size_z", z_size, 2.0);
        nh.param("sdf_map/obstacles_inflation", mp_->obstacles_inflation_, 0.2);
        nh.param("sdf_map/local_bound_inflate", mp_->local_bound_inflate_, 0.5);
        nh.param("sdf_map/ground_height", mp_->ground_height_, 0.4);

        nh.param("camera_util/max_dist", mp_->max_ray_length_, 5.0);
        nh.param("map_manager/map_ground_filter_hight", map_hight_min, 0.4);

        mp_->local_bound_inflate_ = std::max(mp_->resolution_, mp_->local_bound_inflate_);
        mp_->resolution_inv_ = 1.0 / mp_->resolution_;
        mp_->map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_->ground_height_);
        mp_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size - mp_->ground_height_);

        for (Eigen::Index i = 0; i < 3; ++i)
        {
            mp_->map_voxel_num_(i) = ceil(mp_->map_size_(i) / mp_->resolution_);
        }

        mp_->map_min_boundary_ = mp_->map_origin_;
        mp_->map_max_boundary_ = mp_->map_origin_ + mp_->map_size_;

        int buffer_size = getVoxelNum();
        unknown_ = 0, free_ = 10, occupied_ = 15;
        md_->occupancy_buffer_ = std::vector<char>(buffer_size, unknown_);
        md_->occupancy_buffer_inflate_ = std::vector<bool>(buffer_size, false);
        md_->flag_rayend_ = std::vector<int>(buffer_size, 0);
        md_->raycast_num_ = 0;

        buffer_size = get2DVoxelNum();
        md_->explore_buffer_ = std::vector<char>(buffer_size, unknown_);
        // md_->reset_updated_box_ = true;
        
        md_->update_min_ = md_->update_max_ = Eigen::Vector3d(0, 0, 0);
        
        std::vector<std::string> axis = {"x", "y", "z"};
        for (Eigen::Index i = 0; i < 3; ++i) 
        {
            nh.param("sdf_map/box_min_" + axis[i], mp_->box_mind_[i], mp_->map_min_boundary_[i]);
            nh.param("sdf_map/box_max_" + axis[i], mp_->box_maxd_[i], mp_->map_max_boundary_[i]);
        }
        posToIndex(mp_->box_mind_, mp_->box_min_);
        posToIndex(mp_->box_maxd_, mp_->box_max_);


        caster_.reset(new RayCaster);
        caster_->setParams(mp_->resolution_, mp_->map_origin_);
    }


    void SDFMap::initState(Eigen::Vector3d &pos)
    {
        int adr;
        double init_size = mp_->map_size_[2] * 2;
        double resolution = mp_->resolution_ * 0.5;

        for (double x = -init_size; x < init_size + 1e-3; x += resolution)
        {
            for (double y = -init_size; y < init_size + 1e-3; y += resolution)
            {
                for (double z = 0.40; z < 2.5 + 1e-3; z += resolution)
                {

                    Eigen::Vector3d tmp_pos(x+pos[0], y+pos[1], z);
                    if (isInMap(tmp_pos))
                    {
                        Eigen::Vector3i tmp_pos_i;
                        posToIndex(tmp_pos, tmp_pos_i);
                        adr = toAddress(tmp_pos_i);
                        setMapState(adr, FREE);                        
                    }
                }
            }
        }
    }


    void SDFMap::RayUpdateFree(const pcl::PointCloud<pcl::PointXYZ> &points, const Eigen::Vector3d &camera_pos) 
    {
        int point_num = points.size();

        if (point_num == 0)
        {
           return; 
        }

        // setMapState(toAddress(camera_pos[0]), FREE); 

        md_->raycast_num_ += 1;

        Eigen::Vector3d update_min = camera_pos;
        Eigen::Vector3d update_max = camera_pos;

        Eigen::Vector3d pt_w;
        Eigen::Vector3i idx;
        int vox_adr;
        double length;
        for (int i = 0; i < point_num; ++i)             
        {
            auto &pt = points.points[i];
            pt_w << pt.x, pt.y, pt.z;

            if (!isInMap(pt_w))
            {
                pt_w = closetPointInMap(pt_w, camera_pos);
            } 
            length = (pt_w - camera_pos).norm();

            if (length > mp_->max_ray_length_) 
            {
                pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
            }

            posToIndex(pt_w, idx);
            vox_adr = toAddress(idx);                                       // 得到投射射线的终点，起点是相机位姿

            for (int k = 0; k < 3; ++k) 
            {
                update_min[k] = std::min(update_min[k], pt_w[k]);
                update_max[k] = std::max(update_max[k], pt_w[k]);
            }

            if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)            // 防止多次投射
            {
                continue;
            }
            else
            {
                md_->flag_rayend_[vox_adr] = md_->raycast_num_;
            }

            caster_->input(pt_w, camera_pos);
            caster_->nextId(idx);
            while (caster_->nextId(idx))
            {
                setMapState(toAddress(idx), FREE);                          // 射线内的体素设置为空
            }
        }

        Eigen::Vector3d bound_inf(mp_->local_bound_inflate_, mp_->local_bound_inflate_, 0);
        posToIndex(update_max + bound_inf, md_->local_bound_max_);
        posToIndex(update_min - bound_inf, md_->local_bound_min_);
        boundIndex(md_->local_bound_min_);
        boundIndex(md_->local_bound_max_);

        md_->update_min_ = update_min;
        md_->update_max_ = update_max;
    }


    void SDFMap::updateOccupied(const pcl::PointCloud<pcl::PointXYZ> &points)
    {
        int point_num = points.size();

        if (point_num == 0)
        {
            return ;
        }

        Eigen::Vector3d pt_w;
        Eigen::Vector3i idx;
        for (auto &point: points)
        {
            pt_w << point.x, point.y, point.z;
            if (isInMap(pt_w))
            {
                posToIndex(pt_w, idx);
                setMapState(toAddress(idx), OCCUPIED);                
            }
        }
    }

    void SDFMap::deleteOccupied(const pcl::PointCloud<pcl::PointXYZ> &points)
    {
        int point_num = points.size();

        if (point_num == 0)
        {
            return ;
        }

        Eigen::Vector3d pt_w;
        Eigen::Vector3i idx;
        for (auto &point: points)
        {
            pt_w << point.x, point.y, point.z;
            if (isInMap(pt_w))
            {
                posToIndex(pt_w, idx);
                setMapState(toAddress(idx), FREE);                
            }
        }
    }

    void SDFMap::setMapState(const int &adr, const STATE &state)
    {
        if (state == OCCUPIED)
        {
            md_->occupancy_buffer_[adr] = occupied_;
        }
        else if (state == FREE)
        {
            md_->occupancy_buffer_[adr]--;

            if (md_->occupancy_buffer_[adr] <= unknown_)
            {
                md_->occupancy_buffer_[adr] = free_ - 1;
            }
        }
        else
        {
            md_->occupancy_buffer_[adr] = unknown_;
        }
    }


    void SDFMap::resetBuffer() 
    {
        resetBuffer(mp_->map_min_boundary_, mp_->map_max_boundary_);

        md_->local_bound_min_ = Eigen::Vector3i::Zero();
        md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
    }


    void SDFMap::resetBuffer(const Eigen::Vector3d &min_pos, const Eigen::Vector3d &max_pos) 
    {
        Eigen::Vector3i min_id, max_id;
        posToIndex(min_pos, min_id);
        posToIndex(max_pos, max_id);
        boundIndex(min_id);
        boundIndex(max_id);

        for (int x = min_id(0); x <= max_id(0); ++x)
        {
            for (int y = min_id(1); y <= max_id(1); ++y)
            {
                for (int z = min_id(2); z <= max_id(2); ++z) 
                {
                    md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = false;             // 局部膨胀地图置0
                }
            }
        }
    }


    Eigen::Vector3d SDFMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt) 
    {
        Eigen::Vector3d diff = pt - camera_pt;
        Eigen::Vector3d max_tc = mp_->map_max_boundary_ - camera_pt;
        Eigen::Vector3d min_tc = mp_->map_min_boundary_ - camera_pt;

        double min_t = std::numeric_limits<double>::max();
        for (Eigen::Index i = 0; i < 3; ++i) {
            if (fabs(diff[i]) > 0) 
            {
                double t1 = max_tc[i] / diff[i];
                if (t1 > 0 && t1 < min_t) min_t = t1;
                double t2 = min_tc[i] / diff[i];
                if (t2 > 0 && t2 < min_t) min_t = t2;
            }
        }
        return camera_pt + (min_t - 1e-3) * diff;
    }


    void SDFMap::inflateLocalMap()
    {
        int inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);

        Eigen::Vector3i update_min, update_max, update_max_inf, update_min_inf;
        posToIndex(md_->update_min_, update_min);
        posToIndex(md_->update_max_, update_max);
        update_min_inf = md_->local_bound_min_;
        update_max_inf = md_->local_bound_max_;
        std::vector<Eigen::Vector3i> points;

        // 写锁
        // std::unique_lock<std::mutex> lock(md_->occupancy_mutex_);

        // 中间区域
        for (int x = update_min[0]; x <= update_max[0]; x++)
        {
            for (int y = update_min[1]; y <= update_max[1]; y++)
            {
                for (int z = update_min[2]; z <= update_max[2]; z++) 
                {
                    int id1 = toAddress(x, y, z);
                    
                    if (md_->occupancy_buffer_[id1] > free_)
                    {
                        points.emplace_back(Eigen::Vector3i(x, y, z));
                    } 
                    else
                    {
                        md_->occupancy_buffer_inflate_[id1] = false;            // FIXED
                    }
                }
            }
        }

        // 上边界
        for (int x = update_min_inf[0]; x <= update_max_inf[0]; x++)
        {
            for (int y = update_max[1] + 1; y <= update_max_inf[1]; y++)
            {
                for (int z = update_min[2]; z <= update_max[2]; z++)
                {
                    int id1 = toAddress(x, y, z);
                    
                    if (md_->occupancy_buffer_[id1] > free_)
                    {
                        points.emplace_back(Eigen::Vector3i(x, y, z));
                    } 
                }
            }
        }

        // 下边界
        for (int x = update_min_inf[0]; x <= update_max_inf[0]; x++)
        {
            for (int y = update_min_inf[1]; y <= update_min[1] - 1; y++)
            {
                for (int z = update_min[2]; z <= update_max[2]; z++)
                {
                    int id1 = toAddress(x, y, z);
                    
                    if (md_->occupancy_buffer_[id1] > free_)
                    {
                        points.emplace_back(Eigen::Vector3i(x, y, z));
                    } 
                }
            }
        }

        // 左边界
        for (int x = update_min_inf[0]; x <= update_min[0] - 1; x++)
        {
            for (int y = update_min[1]; y <= update_max[1]; y++)
            {
                for (int z = update_min[2]; z <= update_max[2]; z++)
                {
                    int id1 = toAddress(x, y, z);
                    
                    if (md_->occupancy_buffer_[id1] > free_)
                    {
                        points.emplace_back(Eigen::Vector3i(x, y, z));
                    } 
                }
            }
        }

        // 右边界
        for (int x = update_max[0] + 1; x <= update_max_inf[0]; x++)
        {
            for (int y = update_min[1]; y <= update_max[1]; y++)
            {
                for (int z = update_min[2]; z <= update_max[2]; z++)
                {
                    int id1 = toAddress(x, y, z);
                    
                    if (md_->occupancy_buffer_[id1] > free_)
                    {
                        points.emplace_back(Eigen::Vector3i(x, y, z));
                    } 
                }
            }
        }

        int z = 0;
        for (auto &point: points)
        {
            for (int x = -inf_step; x <= inf_step; x++)
            {
                for (int y = -inf_step; y <= inf_step; y++)
                {
                    int idx = toAddress((point + Eigen::Vector3i(x, y, z)));
                    md_->occupancy_buffer_inflate_[idx] = true;
                }
            }
        }
    }


    void SDFMap::updateExploreMap()
    {
        Eigen::Vector3i update_max_inf, update_min_inf;
        update_min_inf = md_->local_bound_min_;
        update_max_inf = md_->local_bound_max_;

        // 写锁
        std::unique_lock<std::mutex> lock(md_->explore_mutex_);

        int id1;
        bool occupanied_flag, free_flag;
        for (int x = update_min_inf[0]; x <= update_max_inf[0]; x++)
        {
            for (int y = update_min_inf[1]; y <= update_max_inf[1]; y++)
            {
                occupanied_flag = false;
                free_flag = false;
                
                // 扫描z轴，如果z轴存在占据，则占据；如果不为占据，存在空，则为空；如果全为未知，则为未知；
                for (int z = update_min_inf[2]; z <= update_max_inf[2]; z++)
                {
                    id1 = toAddress(x, y, z);

                    // 膨胀包括不膨胀部分
                    if (md_->occupancy_buffer_inflate_[id1])
                    {
                        occupanied_flag = true;
                        break;
                    }

                    if (md_->occupancy_buffer_[id1] != unknown_)
                    {
                        free_flag = true;
                        break;
                    }
                }

                id1 = to2DAddress(x, y);
                if (occupanied_flag)
                {
                    md_->explore_buffer_[id1] = occupied_;
                }
                else
                {
                    if (free_flag)
                    {
                        md_->explore_buffer_[id1] = free_;
                    }
                }
            }
        }
    }


    
    void SDFMap::getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax)
    {
        bmin = md_->update_min_;
        bmax = md_->update_max_;

        // std::cout << "update_min: " << md_->update_min_.transpose() << std::endl;
        // std::cout << "update_max: " << md_->update_max_.transpose() << std::endl;
    }


    double SDFMap::getResolution() 
    {
        return mp_->resolution_;
    }

    
    double SDFMap::getGroundHeight() 
    {
        return mp_->ground_height_;
    }

    
    double SDFMap::getMapSizeZ() 
    {
        return (mp_->map_size_.z() + mp_->ground_height_);
    }


    int SDFMap::getVoxelNum() 
    {
        return mp_->map_voxel_num_[0] * mp_->map_voxel_num_[1] * mp_->map_voxel_num_[2];
    }

    int SDFMap::get2DVoxelNum() 
    {
        return mp_->map_voxel_num_[0] * mp_->map_voxel_num_[1];
    }


    void SDFMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size) 
    {
        ori = mp_->map_origin_, size = mp_->map_size_;
    }


    void SDFMap::getBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax) 
    {
        bmin = mp_->box_mind_;
        bmax = mp_->box_maxd_;
    }

}