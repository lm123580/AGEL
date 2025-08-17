#include "sdf_map.h"
#include "safe_flight_corridor.h"

namespace AGEL
{
    SFC::SFC(ros::NodeHandle &nh, const std::shared_ptr<SDFMap> &map)
    {
        nh.param("safe_flight_corridor/max_line_length", max_line_length_, 2.0);
        nh.param("safe_flight_corridor/max_sampling_length", max_sampling_length_, 3.0);

        max_sampling_length_squared_ = max_sampling_length_ * max_sampling_length_;

        map_ = map;

        map_resolution_ = map_->getResolution();
        map_min_ = map_->mp_->map_min_boundary_;
        map_max_ = map_->mp_->map_max_boundary_;

        hPolysPub_ = std::make_shared<ros::Publisher>(nh.advertise<decomp_ros_msgs::PolyhedronArray>("/global/h_polys", 1));
    }

    SFC::~SFC() = default;


    void SFC::generateSFC(const std::vector<Eigen::Vector3d> &path)
    {
        assert(path.size() > 1);
        std::vector<int> path2splited_idx;
        vec_Vec3f non_splited_path, splited_path, point_cloud;
        pathResampling(path, path2splited_idx, splited_path);

        // 得到路径点对应的多面体索引
        path2hPloy_idx_ = std::move(path2splited_idx);

        // 得到路径点附近的点云
        getPointCloudAroundLine(path, point_cloud);

        // 根据分割后的路径点和路径点附近的点云生成多面体
        EllipsoidDecomp3D decomp_util;
        vec_E<Polyhedron3D> polyhedrons;
        
        decomp_util.set_local_bbox(Eigen::Vector3d(max_line_length_, max_line_length_, max_line_length_ * 0.5));
        decomp_util.set_obs(point_cloud);
        decomp_util.dilate(splited_path);

        polyhedrons = decomp_util.get_polyhedrons();

        // 得到多面体的超平面上的一个点及其法向量(该向量指向多面体外部)
        hPolys_.clear();
        int decompPolys_size = polyhedrons.size();
        int current_hyperplanes_size;
        Eigen::MatrixXd current_poly;
        for (int i = 0; i < decompPolys_size; i++) 
        {
            vec_E<Hyperplane3D> current_hyperplanes = polyhedrons[i].hyperplanes();
            current_hyperplanes_size = current_hyperplanes.size();
            current_poly.resize(6, current_hyperplanes_size);
            for (int j = 0; j < current_hyperplanes_size; j++) 
            {
                current_poly.col(j) << current_hyperplanes[j].n_, current_hyperplanes[j].p_;
            }

            compressPoly(current_poly, 0.4);
            hPolys_.push_back(current_poly);
        }
    }


    void SFC::pathResampling(const std::vector<Eigen::Vector3d> &ref_path, std::vector<int> &ref2splited_idx, vec_Vec3f &splited_path)
    {
        splited_path.clear();
        std::vector<int> non_splited_idx, splited_idx;
        vec_Vec3f non_splited_path;
        
        // 根据路径方向得到分割前的路径，并记录原始路径与分割路径前路径的索引映射
        int path_size = ref_path.size();
        Eigen::Vector3d last_dir = (ref_path[1] - ref_path[0]).normalized(), tmp_dir;
        non_splited_idx.emplace_back(0);
        non_splited_path.emplace_back(ref_path.front());
        for (int i = 2; i < path_size; i++)
        {
            tmp_dir = (ref_path[i] - ref_path[i-1]).normalized();

            if (std::fabs(tmp_dir[0] - last_dir[0]) > 1e-3 || 
                std::fabs(tmp_dir[1] - last_dir[1]) > 1e-3 || 
                std::fabs(tmp_dir[2] - last_dir[2]) > 1e-3)
            {
                last_dir = tmp_dir;
                non_splited_idx.emplace_back(i-1);
                non_splited_path.emplace_back(ref_path[i-1]);
            }
        }
        non_splited_idx.emplace_back(path_size-1);
        non_splited_path.emplace_back(ref_path.back());
        
        // 根据路径长度分割路径，并记录原始路径与分割路径路径的索引映射
        path_size = non_splited_path.size();
        splited_idx.emplace_back(0);
        splited_path.emplace_back(non_splited_path.front());
        double path_dis;
        vec_Vec3f tmp_path;
        for (int i = 1; i < path_size; i++)
        {
            path_dis = (non_splited_path[i] - non_splited_path[i-1]).norm();
            
            if (path_dis > max_sampling_length_)
            {
                int seg_num = std::ceil((path_dis / (max_sampling_length_)));
                int add_idx = std::ceil(((non_splited_idx[i] - non_splited_idx[i-1]) / seg_num));

                for (int j = non_splited_idx[i-1] + add_idx; j <= non_splited_idx[i]; j += add_idx)
                {
                    splited_idx.emplace_back(j);
                    splited_path.emplace_back(ref_path[j]);
                }

                if (splited_idx.back() < non_splited_idx[i])
                {
                    splited_idx.emplace_back(non_splited_idx[i]);
                    splited_path.emplace_back(ref_path[non_splited_idx[i]]);
                }
            }
            else
            {
                splited_idx.emplace_back(non_splited_idx[i]);
                splited_path.emplace_back(ref_path[non_splited_idx[i]]);
            }
        }

        ref2splited_idx.clear();
        path_size = ref_path.size();
        int idx = 0;
        for (int i = 0; i < path_size; i++)
        {
            ref2splited_idx.emplace_back(idx);
            if (i == splited_idx[idx+1])
            {
                idx++;
            }
        }
        // ref2splited_idx = std::move(splited_idx);
    }


    void SFC::getPointCloudAroundLine(const std::vector<Eigen::Vector3d> &path, vec_Vec3f &point_cloud)
    {
        point_cloud.clear();

        Eigen::Vector3d min_pos, max_pos;
        min_pos << std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max();
        max_pos << -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max();

        for (const auto &p: path)
        {
            if (p[0] < min_pos[0]) min_pos[0] = p[0];
            if (p[1] < min_pos[1]) min_pos[1] = p[1];
            if (p[0] > max_pos[0]) max_pos[0] = p[0];
            if (p[1] > max_pos[1]) max_pos[1] = p[1];
        }

        min_pos[0] -= max_line_length_;
        min_pos[1] -= max_line_length_;
        max_pos[0] += max_line_length_;
        max_pos[1] += max_line_length_;

        for (double x = min_pos[0]; x <= max_pos[0]; x += map_resolution_)
        {
            for (double y = min_pos[1]; y <= max_pos[1]; y += map_resolution_)
            {
                for (double z = map_min_[2]; z <= map_max_[2]; z += map_resolution_)
                {
                    Eigen::Vector3d tmp_p(x, y, z);
                    if (map_->getOccupancy(tmp_p) == map_->OCCUPIED)
                    {
                        point_cloud.emplace_back(tmp_p);
                    }
                }
            }
        }
    }


    void SFC::compressPoly(Polyhedron3D& poly, double dx) 
    {
        vec_E<Hyperplane3D> hyper_planes = poly.hyperplanes();

        for (uint j = 0; j < hyper_planes.size(); j++) 
        {
            hyper_planes[j].p_ = hyper_planes[j].p_ - hyper_planes[j].n_ * dx;
        }
        
        poly = Polyhedron3D(hyper_planes);
    }

    void SFC::compressPoly(Eigen::MatrixXd& poly, double dx) 
    {
        for (int i = 0; i < poly.cols(); ++i) 
        {
            poly.col(i).tail(3) = poly.col(i).tail(3) - poly.col(i).head(3) * dx;
        }
    }


    void SFC::visCorridor(const vec_E<Polyhedron3D> &polyhedra) 
    {
        decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polyhedra);
        poly_msg.header.frame_id = "map";
        poly_msg.header.stamp = ros::Time::now();
        
        hPolysPub_->publish(poly_msg);
    }
    

    void SFC::visCorridor() 
    {
        vec_E<Polyhedron3D> decompPolys;
        
        for (const auto &poly : hPolys_) 
        {
            vec_E<Hyperplane3D> hyper_planes;
            hyper_planes.resize(poly.cols());
            
            for (int i = 0; i < poly.cols(); ++i) 
            {
                hyper_planes[i].n_ = poly.col(i).head(3);
                hyper_planes[i].p_ = poly.col(i).tail(3);
            }
            
            decompPolys.emplace_back(hyper_planes);
        }
        
        visCorridor(decompPolys);
    }


    void SFC::visCorridor(const std::vector<Eigen::MatrixXd> &hPolys) 
    {
        vec_E<Polyhedron3D> decompPolys;
        
        for (const auto &poly : hPolys) 
        {
            vec_E<Hyperplane3D> hyper_planes;
            hyper_planes.resize(poly.cols());
            
            for (int i = 0; i < poly.cols(); ++i) 
            {
                hyper_planes[i].n_ = poly.col(i).head(3);
                hyper_planes[i].p_ = poly.col(i).tail(3);
            }
            
            decompPolys.emplace_back(hyper_planes);
        }

        visCorridor(decompPolys);
    }


    void SFC::getCorridorPolys(std::vector<Eigen::MatrixXd> &polys) 
    {
        polys = hPolys_;
    }

    void SFC::getpath2hPloyIdx(std::vector<int> &path2hPloy_idx)
    {
        path2hPloy_idx = path2hPloy_idx_;
    }
}