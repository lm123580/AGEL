#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>

namespace AGEL
{
    class UniformBspline
    {
    private:
        Eigen::MatrixXd control_points_;    // 控制点 n+1 * p
        Eigen::MatrixXd v_control_points_;  
        Eigen::MatrixXd a_control_points_;  
        Eigen::Index p_{}, n_{}, m_{};      // p 次, n+1 控制点数量, m = n+p+1
        Eigen::VectorXd u_;                 // 节点向量，数量为m+1
        double knot_span_{};                // 节点间隔 \delta t

    public:
        UniformBspline();

        UniformBspline(const Eigen::MatrixXd &points, const int &degree, const double &interval);
        
        ~UniformBspline();

        // 初始化一个均匀B样条
        void setUniformBspline(const Eigen::MatrixXd &points, const int &degree, const double &interval);

        // 设置B样条节点向量
        void setKnot(const Eigen::VectorXd &knot);

        // 得到B样条节点向量
        Eigen::VectorXd getKnot();

        // 得到B样条控制点
        Eigen::MatrixXd getControlPoints();

        Eigen::MatrixXd getVControlPoints();

        Eigen::MatrixXd getAControlPoints();

        // 得到节点间间隔
        double getKnotSpan() const;

        void setKnotSpan(double knot_span);

        // 得到总的时间间隔
        void getTimeSpan(double &um, double &um_p);

        // 时间参数化B样条，并得到控制点
        // constraints
        // input : (K+2) points with boundary vel/acc; ts
        // output: (K+6) control_pts
        static void parameterizeToBspline(const double &ts, const std::vector<Eigen::Vector3d> &waypoints,
                                          const std::vector<Eigen::Vector3d> &start_end_derivative,
                                          const int &degree, Eigen::MatrixXd &ctrl_pts);
        
        // 弧长参数化当前B样条
        std::vector<Eigen::Vector3d> parameterizeToArcBspline(int segme);

        // 根据DeBoor算法计算B样条值
        Eigen::VectorXd evaluateDeBoor(const double &u);
        Eigen::VectorXd evaluateDeBoorT(const double &t);
        
        // B样条求导
        Eigen::MatrixXd getDerivativeCtrlPts(const Eigen::MatrixXd &ctrl_pts);

        // 快速计算B样条值
        Eigen::VectorXd getBsplineValueFast(const double &t, const Eigen::MatrixXd &ctrl_pts, const int degree, Eigen::VectorXd &g);
        // template <class T> static T getBsplineValueFast(const double ts, const Eigen::MatrixXd &ctrl_pts, double t, int degree, T *grad = nullptr);

        void initDerivativeCtrlPts();

        void saveInfo(std::string &data_name, double &sample_num);

        double getTimeSum();

        double getLength(double res);

        void getSamplePoints(std::vector<Eigen::Vector3d> &sample_points, double &res);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}



#endif