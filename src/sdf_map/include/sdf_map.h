#ifndef _SDF_MAP_H_
#define _SDF_MAP_H_

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <queue>

#include <mutex>

class RayCaster;

namespace AGEL
{
    struct MapParam 
    {
        Eigen::Vector3d map_origin_, map_size_;                 // 地图基本参数
        Eigen::Vector3d map_min_boundary_, map_max_boundary_;
        Eigen::Vector3i map_voxel_num_;
        double resolution_, resolution_inv_;
        double obstacles_inflation_;
        double virtual_ceil_height_, ground_height_;
        Eigen::Vector3i box_min_, box_max_;                     // 地图AABB
        Eigen::Vector3d box_mind_, box_maxd_;

        double max_ray_length_;                                 // 光线投射长度
        double local_bound_inflate_;                            

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };


    struct MapData 
    {
        std::mutex occupancy_mutex_;                        // 互斥锁
        std::mutex explore_mutex_;                          // 互斥锁

        std::vector<char> occupancy_buffer_;                // 栅格地图数据
        std::vector<bool> occupancy_buffer_inflate_;

        std::vector<char> explore_buffer_;                   // 探索二维地图数据

        std::vector<int> flag_rayend_;
        char raycast_num_;
        Eigen::Vector3i local_bound_min_, local_bound_max_;
        Eigen::Vector3d update_min_, update_max_;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };


    class SDFMap
    {
    public:
        enum STATE
        {
            UNKNOWN, OCCUPIED, FREE
        };
        std::shared_ptr<MapParam> mp_;
        std::shared_ptr<MapData>  md_;

        int unknown_, occupied_, free_;
        
    private:
        std::shared_ptr<RayCaster> caster_;
        
        void inflatePoint(const Eigen::Vector3i &pt, int step, std::vector<Eigen::Vector3i> &pts);

        void setMapState(const int &adr, const STATE &state);

        Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);

    public:
        void initMap(ros::NodeHandle &nh);

        void initState(Eigen::Vector3d &pos);

        void inflateLocalMap();

        void updateExploreMap();

        void RayUpdateFree(const pcl::PointCloud<pcl::PointXYZ> &points, const Eigen::Vector3d &camera_pos);
        
        void updateOccupied(const pcl::PointCloud<pcl::PointXYZ> &points);

        void deleteOccupied(const pcl::PointCloud<pcl::PointXYZ> &points);

        void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);

        void posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id);

        void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);

        void indexToPos(const int &idx, Eigen::Vector3d &pos);

        void indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos);

        void boundIndex(Eigen::Vector3i &id);

        int toAddress(const Eigen::Vector3i &id);

        int toAddress(const int &x, const int &y, const int &z);

        int to2DAddress(const int &x, const int &y);

        bool isInMap(const Eigen::Vector3d &pos);

        bool isInMap(const Eigen::Vector3i &idx);

        bool isInMap(const int &idx);

        bool isInBox(const Eigen::Vector3i &id);

        bool isInBox(const Eigen::Vector3d &pos);

        void boundBox(Eigen::Vector3d &low, Eigen::Vector3d &up);

        int getOccupancy(const Eigen::Vector3d &pos);

        int getOccupancy(const Eigen::Vector3i &id);

        int get2DState(const Eigen::Vector2i &id);

        bool getInflateOccupancy(const Eigen::Vector3d &pos);

        bool getInflateOccupancy(const Eigen::Vector3i &id);

        void resetBuffer();

        void resetBuffer(const Eigen::Vector3d &min, const Eigen::Vector3d &max);

        void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);

        void getBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax);

        void getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax);

        double getResolution();

        double getGroundHeight();

        double getMapSizeZ();

        int getVoxelNum();

        int get2DVoxelNum();

        SDFMap();

        ~SDFMap();

        // typedef std::shared_ptr<SDFMap> Ptr;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };


    inline void SDFMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id) 
    {
        for (size_t i = 0; i < 3; ++i)
        {
            id(i) = floor((pos(i) - mp_->map_origin_(i)) * mp_->resolution_inv_);
        }
    }

    inline void SDFMap::posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id) 
    {
        for (size_t i = 0; i < 2; ++i)
        {
            id(i) = floor((pos(i) - mp_->map_origin_(i)) * mp_->resolution_inv_);
        }
    }


    inline void SDFMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos) 
    {
        for (size_t i = 0; i < 3; ++i)
        {
            pos(i) = (id(i) + 0.5) * mp_->resolution_ + mp_->map_origin_(i);
        }
    }


    inline void SDFMap::indexToPos(const int &idx, Eigen::Vector3d &pos) 
    {
        Eigen::Vector3i id((idx / (mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2))), 
                           ((idx % (mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2))) / mp_->map_voxel_num_(2)), 
                           (idx % mp_->map_voxel_num_(2)));
        
        for (size_t i = 0; i < 3; ++i)
        {
            pos(i) = (id(i) + 0.5) * mp_->resolution_ + mp_->map_origin_(i);
        }
    }


    inline void SDFMap::indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos) 
    {
        for (size_t i = 0; i < 2; ++i)
        {
            pos(i) = (id(i) + 0.5) * mp_->resolution_ + mp_->map_origin_(i);
        }
    }


    inline void SDFMap::boundIndex(Eigen::Vector3i &id) 
    {
        Eigen::Vector3i id_tmp;

        id_tmp(0) = std::max(std::min(id(0), mp_->map_voxel_num_(0) - 1), 0);
        id_tmp(1) = std::max(std::min(id(1), mp_->map_voxel_num_(1) - 1), 0);
        id_tmp(2) = std::max(std::min(id(2), mp_->map_voxel_num_(2) - 1), 0);

        id = id_tmp;
    }


    inline int SDFMap::to2DAddress(const int &x, const int &y)
    {
        return x * mp_->map_voxel_num_(1) + y;
    }


    inline int SDFMap::toAddress(const int &x, const int &y, const int &z) 
    {
        return x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z;
    }


    inline int SDFMap::toAddress(const Eigen::Vector3i &id) 
    {
        return toAddress(id[0], id[1], id[2]);
    }


    inline bool SDFMap::isInMap(const Eigen::Vector3d &pos) 
    {
        if (pos(0) < mp_->map_min_boundary_(0) + 1e-4 || pos(1) < mp_->map_min_boundary_(1) + 1e-4 || 
            pos(2) < mp_->map_min_boundary_(2) + 1e-4 || pos(0) > mp_->map_max_boundary_(0) - 1e-4 || 
            pos(1) > mp_->map_max_boundary_(1) - 1e-4 || pos(2) > mp_->map_max_boundary_(2) - 1e-4)
        {
            return false;
        }
        else
        {
            return true;
        }
    }


    inline bool SDFMap::isInMap(const Eigen::Vector3i &idx) 
    {
        Eigen::Vector3d pos;
        indexToPos(idx, pos);

        return isInMap(pos);
        // if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0 || 
        //     idx(0) > mp_->map_voxel_num_(0) - 1 || 
        //     idx(1) > mp_->map_voxel_num_(1) - 1 || 
        //     idx(2) > mp_->map_voxel_num_(2) - 1)
        // {
        //    return false; 
        // }
        // else
        // {
        //     return true;
        // }
    }

    inline bool SDFMap::isInMap(const int &idx)
    {
        Eigen::Vector3d pos;
        indexToPos(idx, pos);

        return isInMap(pos);
        // if (idx > mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) - 1 || idx < 0)
        // {
        //    return false; 
        // }
        // else
        // {
        //     return true;
        // }
    }


    inline bool SDFMap::isInBox(const Eigen::Vector3i &id) 
    {
        for (size_t i = 0; i < 3; ++i) 
        {
            if (id[i] <= mp_->box_min_[i] || id[i] >= mp_->box_max_[i]) 
            {
                return false;
            }
        }

        return true;
    }


    inline bool SDFMap::isInBox(const Eigen::Vector3d &pos) 
    {
        for (size_t i = 0; i < 3; ++i) 
        {
            if (pos[i] <= mp_->box_mind_[i] || pos[i] >= mp_->box_maxd_[i]) 
            {
                return false;
            }
        }
        
        return true;
    }


    inline void SDFMap::boundBox(Eigen::Vector3d &low, Eigen::Vector3d &up) 
    {
        for (size_t i = 0; i < 3; ++i) 
        {
            low[i] = std::max(low[i], mp_->box_mind_[i]);
            up[i] = std::min(up[i], mp_->box_maxd_[i]);
        }
    }


    inline int SDFMap::getOccupancy(const Eigen::Vector3i &id) 
    {
        if (!isInMap(id))
        {
            return OCCUPIED;
        }
        
        if (md_->occupancy_buffer_[toAddress(id)] == unknown_)
        {
            return UNKNOWN;
        }
        else if (md_->occupancy_buffer_[toAddress(id)] >= free_)
        {
            return OCCUPIED;
        }
        else
        {
            return FREE;
        }
    }


    inline int SDFMap::getOccupancy(const Eigen::Vector3d &pos) 
    {
        Eigen::Vector3i id;
        posToIndex(pos, id);

        return getOccupancy(id);
    }


    inline int SDFMap::get2DState(const Eigen::Vector2i &id)
    {
        int adr = to2DAddress(id[0], id[1]);

        if (md_->explore_buffer_[adr] == unknown_)
        {
            return UNKNOWN;
        }
        else if (md_->explore_buffer_[adr] == occupied_)
        {
            return OCCUPIED;
        }
        else
        {
            return FREE;
        }
    }


    inline bool SDFMap::getInflateOccupancy(const Eigen::Vector3i &id) 
    {
        if (!isInMap(id))
        {
            return true;
        }

        return md_->occupancy_buffer_inflate_[toAddress(id)];
    }


    inline bool SDFMap::getInflateOccupancy(const Eigen::Vector3d &pos) 
    {
        Eigen::Vector3i id;
        posToIndex(pos, id);

        return getInflateOccupancy(id);
    }


    inline void SDFMap::inflatePoint(const Eigen::Vector3i &pt, int step, std::vector<Eigen::Vector3i> &pts) 
    {
        pts.clear();

        /* ---------- all inflate ---------- */
        // pts.emplace_back(0, 0, 0);

        // int direct_size = all_direct_.size();
        // for (int i = 0; i < direct_size; i++)
        // {
        //     for (int j = 1; j <= step; j++)
        //     {
        //         Eigen::Vector3i tmp = pt + all_direct_[i] * j;

        //         if (getInflateOccupancy(tmp))   break;
        //         else                            pts.emplace_back(tmp);
        //     }
        // }
    }
}

#endif
