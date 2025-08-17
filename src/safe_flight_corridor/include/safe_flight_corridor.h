#ifndef SAFE_FLIGHT_COORRIDOR_H_
#define SAFE_FLIGHT_COORRIDOR_H_

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <memory>
#include <chrono>

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_ros_utils/data_ros_utils.h>

namespace AGEL
{
    class SDFMap;

    class SFC
    {
    private:
        std::shared_ptr<SDFMap> map_;
        std::shared_ptr<EllipsoidDecomp3D> ellipsoid_decomp_;

        double max_line_length_;
        double max_sampling_length_;
        double max_sampling_length_squared_;
        double map_resolution_;
        Eigen::Vector3d map_min_, map_max_;
        std::vector<int> path2hPloy_idx_;             // 路径点->多面体索引    
        std::vector<Eigen::MatrixXd> hPolys_;         // 高维多面体集合
        std::shared_ptr<ros::Publisher> hPolysPub_;


        void pathResampling(const std::vector<Eigen::Vector3d> &ref_path,  std::vector<int> &ref2splited_idx, vec_Vec3f &splited_path);

        void getPointCloudAroundLine(const std::vector<Eigen::Vector3d> &path, vec_Vec3f &point_cloud);

        void compressPoly(Polyhedron3D& poly, double dx);

        void compressPoly(Eigen::MatrixXd& poly, double dx);

        void visCorridor(const vec_E<Polyhedron3D> &polyhedra);

    public:
        SFC(ros::NodeHandle &nh, const std::shared_ptr<SDFMap> &map);

        ~SFC();

        void generateSFC(const std::vector<Eigen::Vector3d>& path);

        void getCorridorPolys(std::vector<Eigen::MatrixXd> &polys);

        void getpath2hPloyIdx(std::vector<int> &path2hPloy_idx);

        void visCorridor();

        void visCorridor(const std::vector<Eigen::MatrixXd>& hPolys);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}

#endif