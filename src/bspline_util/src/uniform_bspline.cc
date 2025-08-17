#include "uniform_bspline.h"
#include <chrono>

namespace AGEL
{
    UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &degree, const double &interval) 
    {
        setUniformBspline(points, degree, interval);
    }

    UniformBspline::UniformBspline() = default;

    UniformBspline::~UniformBspline() = default;

    void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &degree, const double &interval) {
        control_points_ = points;
        p_ = degree;
        knot_span_ = interval;

        n_ = points.rows() - 1;
        m_ = n_ + p_ + 1;

        u_ = Eigen::VectorXd::Zero(m_ + 1);
        for (Eigen::Index i = 0; i <= m_; ++i) 
        {
            if (i <= p_)
            {
                u_(i) = double(-p_ + i) * knot_span_;
            }
            else
            {
                u_[i] = u_[i - 1] + knot_span_;
            }
        }
    }

    void UniformBspline::setKnot(const Eigen::VectorXd &knot) 
    {
        u_ = knot;
    }

    Eigen::VectorXd UniformBspline::getKnot() 
    {
        return u_;
    }


    void UniformBspline::getTimeSpan(double &um, double &um_p) 
    {
        um = u_(p_);
        um_p = u_(m_ - p_);
    }


    Eigen::MatrixXd UniformBspline::getControlPoints() 
    {
        return control_points_;
    }

    Eigen::MatrixXd UniformBspline::getVControlPoints() 
    {
        return v_control_points_;
    } 

    Eigen::MatrixXd UniformBspline::getAControlPoints() 
    {
        return a_control_points_;
    } 

    double UniformBspline::getKnotSpan() const 
    {
        return knot_span_;
    }

    void UniformBspline::setKnotSpan(double knot_span) 
    {
        knot_span_ = knot_span;
    }

    void UniformBspline::parameterizeToBspline( const double &ts, const std::vector<Eigen::Vector3d> &waypoints,
                                                const std::vector<Eigen::Vector3d> &start_end_derivative,
                                                const int &degree, Eigen::MatrixXd &ctrl_pts) 
    {
        if (ts <= 0) 
        {
            std::cout << "[B-spline]:time step error." << std::endl;
            return;
        }
        
        if (waypoints.size() < 2) 
        {
            std::cout << "[B-spline]:point set have only " << waypoints.size() << " points." << std::endl;
            return;
        }

        if (start_end_derivative.size() != 4) 
        {
            std::cout << "[B-spline]:derivatives error." << std::endl;
        }

        // 航点数量
        auto K = (Eigen::Index) waypoints.size();

        // 求解通过航点的B样条的控制点
        // 矩阵细节来自于论文 "General matrix representations for B-splines, Qin, Kaihuai"
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + degree - 1);
        // std::cout << "A size\t" << A.size() << std::endl;
        Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
        ctrl_pts.resize(K + degree - 1, 3);

        if (degree == 3) 
        {
            // 将控制点通过矩阵映射到航点和边界导数 
            Eigen::Vector3d pt_to_pos = 1 / 6.0 * Eigen::Vector3d(1, 4, 1);
            Eigen::Vector3d pt_to_vel = 1 / (2.0 * ts) * Eigen::Vector3d(-1, 0, 1);
            Eigen::Vector3d pt_to_acc = 1 / (ts * ts) * Eigen::Vector3d(1, -2, 1);

            for (Eigen::Index i = 0; i < K; ++i)
            {
                A.block<1, 3>(i, i) = pt_to_pos.transpose();
            }
            A.block<1, 3>(K, 0) = pt_to_vel.transpose();
            A.block<1, 3>(K + 1, K - 1) = pt_to_vel.transpose();
            A.block<1, 3>(K + 2, 0) = pt_to_acc.transpose();
            A.block<1, 3>(K + 3, K - 1) = pt_to_acc.transpose();
        }

        // K个航点和4个边界导数
        for (Eigen::Index i = 0; i < K; ++i) 
        {
            bx(i) = waypoints[i][0];
            by(i) = waypoints[i][1];
            bz(i) = waypoints[i][2];
        }
        
        for (Eigen::Index i = 0; i < 4; ++i) 
        {
            bx(K + i) = start_end_derivative[i][0];
            by(K + i) = start_end_derivative[i][1];
            bz(K + i) = start_end_derivative[i][2];
        }

        // 求解 Ax = b 得到控制点x
        ctrl_pts.col(0) = A.colPivHouseholderQr().solve(bx);
        ctrl_pts.col(1) = A.colPivHouseholderQr().solve(by);
        ctrl_pts.col(2) = A.colPivHouseholderQr().solve(bz);
    }


    Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u)
    {
        double ub = std::min(std::max(u_(p_), u), u_(m_ - p_));

        // Determine which [uk,uk+1] does u lay in
        Eigen::Index k = p_;
        while (k + 1 < u_.rows() && u_(k + 1) < ub)
        {
            ++k;
        }

        /* deBoor's algorithm */
        // [uk,uk+1] is controlled by q[k-p]...q[k], retrieve the associated points
        std::vector<Eigen::VectorXd> d;
        for (Eigen::Index i = 0; i <= p_; ++i)
        {
            d.emplace_back(control_points_.row(k - p_ + i));
        }

        double alpha;
        for (Eigen::Index r = 1; r <= p_; ++r)
        {
            for (Eigen::Index i = p_; i >= r; --i) 
            {
                alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
                d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
            }
        }

        return d[p_];
    }

    Eigen::VectorXd UniformBspline::evaluateDeBoorT(const double &t)
    {
        return evaluateDeBoor(t + u_(p_));
    }

    Eigen::MatrixXd UniformBspline::getDerivativeCtrlPts(const Eigen::MatrixXd &ctrl_pts)
    {
        Eigen::MatrixXd derivative_pts = Eigen::MatrixXd::Zero(ctrl_pts.rows() - 1, ctrl_pts.cols());
        
        for (int i = 0; i < ctrl_pts.rows() - 1; i++) 
        {
            Eigen::Vector3d q1 = ctrl_pts.row(i + 1);
            Eigen::Vector3d q0 = ctrl_pts.row(i);
            derivative_pts.row(i) = (q1 - q0) / knot_span_;
        }

        return derivative_pts;
    }
    

    Eigen::VectorXd UniformBspline::getBsplineValueFast(const double &t, const Eigen::MatrixXd &ctrl_pts, const int degree, Eigen::VectorXd &g)
    {
        Eigen::VectorXd value(ctrl_pts.cols());

        double e_t = std::min(std::max(u_(p_), t), u_(m_ - p_)) / knot_span_ + degree;

        int k = e_t;
        if (k < degree)                     k = degree;
        else if (k > ctrl_pts.rows() - 1)   k = ctrl_pts.rows() - 1;
        
        double x = e_t - k;             // s_u                  or      t
        double x_g = 1.0 / knot_span_;
        double ix = 1.0 - x;            // 4.0 - (s_u + 3.0)    or      4.0 - t
        double ix_g = -x_g;
        double w0, w1, w2, w3;
        double w0_g, w1_g, w2_g, w3_g;

        switch(degree) 
        {
        case 1:
            w0 = ix;
            w1 = x;
            value = w0 * ctrl_pts.row(k - 1) + w1 * ctrl_pts.row(k);
            
            w0_g = ix_g;
            w1_g = x_g;
            g = w0_g * ctrl_pts.row(k - 1) + w1_g * ctrl_pts.row(k);
            break;

        case 2:
            w0 = 0.5 * ix * ix;
            w1 = 0.5 + x - x * x;
            w2 = 0.5 * x * x;
            value = w0 * ctrl_pts.row(k - 2) + w1 * ctrl_pts.row(k - 1) + w2 * ctrl_pts.row(k);
        
            w0_g = ix * ix_g;
            w1_g = x_g - 2 * x * x_g;
            w2_g = x * x_g;
            g = w0_g * ctrl_pts.row(k - 2) + w1_g * ctrl_pts.row(k - 1) + w2_g * ctrl_pts.row(k);
            break;

        case 3:
            w0 = ix * ix * ix / 6.0;
            w1 = 1 / 6.0 + 0.5 * (ix + ix * ix - ix * ix * ix);
            w2 = 1 / 6.0 + 0.5 * (x + x * x - x * x * x);
            w3 = x * x * x / 6.0;
            value = w0 * ctrl_pts.row(k - 3) + w1 * ctrl_pts.row(k - 2) + w2 * ctrl_pts.row(k - 1) + w3 * ctrl_pts.row(k);
            
            w0_g = 0.5 * ix * ix * ix_g;
            w1_g = 0.5 * (ix_g + 2 * ix * ix_g - 3 * ix * ix * ix_g);
            w2_g = 0.5 * (x_g + 2 * x * x_g - 3 * x * x * x_g);
            w3_g = 0.5 * x * x * x_g;
            g = w0_g * ctrl_pts.row(k - 3) + w1_g * ctrl_pts.row(k - 2) + w2_g * ctrl_pts.row(k - 1) + w3_g * ctrl_pts.row(k);
            break;

        default:
            throw "getBsplineValueFast parameter error";
            exit(0);
        }

        return value;
    }

    void  UniformBspline::initDerivativeCtrlPts()
    {
        v_control_points_ = getDerivativeCtrlPts(control_points_);
        a_control_points_ = getDerivativeCtrlPts(v_control_points_);
    }


    double UniformBspline::getTimeSum()
    {
        return u_(m_ - p_) - u_(p_);
    }

    double UniformBspline::getLength(double res) 
    {
        // auto t1 = std::chrono::high_resolution_clock::now();

        double length = 0.0;
        double dur = getTimeSum()- res + 1e-4;
        
        Eigen::VectorXd g;
        for (double t = 0; t <= dur; t += res) 
        {
            getBsplineValueFast(t, control_points_, p_, g);
            length += g.norm() * res;
        }

        // auto t2 = std::chrono::high_resolution_clock::now();
        // std::cout << "length: " << length << std::endl;
        // auto duration_1 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
        // std::cout << "origin: " << duration_1.count() << " us" << std::endl;

        return length;
    }

    void UniformBspline::getSamplePoints(std::vector<Eigen::Vector3d> &sample_points, double &res) 
    {
        double dur = getTimeSum();
        
        sample_points.clear();
        sample_points.push_back(evaluateDeBoorT(0.0));
        Eigen::VectorXd g;
        for (double t = res; t <= dur + 1e-4; t += res) 
        {
            // Eigen::Vector3d temp_1 = evaluateDeBoorT(t);
            Eigen::Vector3d point = getBsplineValueFast(t, control_points_, p_, g);
            sample_points.push_back(point);
        }
    }


    std::vector<Eigen::Vector3d> UniformBspline::parameterizeToArcBspline(int segme)
    {
        // Eigen::MatrixXd control_points_;    // 控制点 n+1+p-1 * p
        // Eigen::Index p_{}, n_{}, m_{};      // p 次, n+1 航点数量, m = n+p+1
        // Eigen::VectorXd u_;                 // 节点向量，数量为m+1
        // double knot_span_{};                // 节点间隔 \delta t
        // int n, p, m;
        int n;
        double length, knot_span, res;
        Eigen::MatrixXd ctrl_pts;
        std::vector<Eigen::Vector3d> waypoints;
        Eigen::VectorXd grad;

        double time_sum = getTimeSum();

        n = segme;
        res = 0.025;
        length = getLength(res);
        knot_span = length / n;

        double cur_time = 0.0;
        // double next_time = 0.0;
        
        // waypoints.push_back(evaluateDeBoorT(0.0));
        waypoints.push_back(getBsplineValueFast(cur_time, control_points_, p_, grad));
        for (int i = 1; i < segme; i++)
        {
            double cur_arc = 0.0;               // 当前弧长
            
            while (cur_arc < knot_span)
            {
                getBsplineValueFast(cur_time, control_points_, p_, grad);
                cur_arc += grad.norm() * res;
                cur_time += res;
                // next_time = next_time + res;
                // cur_arc += (evaluateDeBoorT(next_time) - evaluateDeBoorT(cur_time)).norm();
            }

            if (cur_arc - knot_span >= 1e-6)      // 当前弧长大小与预期不符，使用二分法细分
            {
                cur_time = cur_time - (cur_arc - knot_span) / (grad.norm());
                // // cur_arc -= (evaluateDeBoorT(next_time) - evaluateDeBoorT(cur_time)).norm();
                // double g = grad.norm();
                // cur_arc -= g * res;
                // cur_time -= res;
                // // double mid_t;
                // // double left_t = cur_time;
                // // double right_t = cur_time + res;
                // double mid_t;
                // double left_t = 0.0;
                // double right_t = res;
                // int count = 0;
                // while (true)
                // {
                //     count++;
                //     mid_t = (right_t + left_t) / 2;
                //     // double tmp_l = (evaluateDeBoorT(mid_t) - evaluateDeBoorT(cur_time)).norm();
                //     // double diff_l = cur_arc + tmp_l - knot_span;
                //     double tmp_l = g * mid_t;
                //     double diff_l = 

                //     if (std::abs(diff_l) < 1e-4 || count > 10)   // 退出条件
                //     {
                //         next_time = mid_t;
                //         break;
                //     }

                //     if (diff_l > 0)                             // 选点过远
                //     {
                //         right_t = mid_t;
                //     }
                //     else
                //     {
                //         left_t = mid_t;
                //     }
                // }      
            }

            waypoints.push_back(getBsplineValueFast(cur_time, control_points_, p_, grad));
        }
        waypoints.push_back(getBsplineValueFast(time_sum, control_points_, p_, grad));
        // std::cout << waypoints.size() << std::endl;
        std::vector<Eigen::Vector3d> start_end_derivative;

        // double v = knot_span / 10.0 / std::sqrt(2);
        
        getBsplineValueFast(0.0, control_points_, p_, grad);
        grad = knot_span / 10.0 * grad.normalized();
        start_end_derivative.push_back(Eigen::Vector3d(grad[0], grad[1], grad[2]));
        
        getBsplineValueFast(time_sum, control_points_, p_, grad);
        grad = knot_span / 10.0 * grad.normalized();
        start_end_derivative.push_back(Eigen::Vector3d(grad[0], grad[1], grad[2]));

        start_end_derivative.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        
        start_end_derivative.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        
        parameterizeToBspline(10.0, waypoints, start_end_derivative, p_, ctrl_pts);

        setUniformBspline(ctrl_pts, p_, knot_span);
        
        return waypoints;
    }

    void UniformBspline::saveInfo(std::string &data_name, double &sample_num)
    {
        std::ofstream outputFile;

        outputFile.open(data_name);

        if (!outputFile)
        {
            std::cout << "Error Open File" << std::endl;
            return ;
        }

        outputFile << "x y vel" << std::endl;

        double dur_sum = getTimeSum();
        double dur = dur_sum / sample_num;
        
        double t = 0.0;
        Eigen::VectorXd grad;
        for (int i = 0; i <= sample_num; i++) 
        {
            // Eigen::Vector3d point_cur = evaluateDeBoorT(t);
            Eigen::Vector3d point_cur = getBsplineValueFast(t, control_points_, p_, grad);
            
            t += dur;

            // Eigen::Vector3d point_next = evaluateDeBoorT(t);

            outputFile << point_cur[0] << " " << point_cur[1] << " " << grad.norm() << std::endl;
        }
    }
}