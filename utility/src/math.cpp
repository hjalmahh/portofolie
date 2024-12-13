#include "utility/math.h"
#include "utility/vectors.h"

#include <iostream>
#include <Eigen/Dense>


#include <numeric>

namespace AIS4104::utility {

//----------------------frestyle stuff----------------




//TASK: Implement the following function definitions
    //B1.1 s 557 modern robotics
Eigen::Vector3d euler_zyx_from_rotation_matrix(const Eigen::Matrix3d &r)
{

    double a = 0.0;
    double b = 0.0;
    double c = 0.0;

    if(is_approx_equal(r(2, 0), -1.0))
    {
        b = EIGEN_PI / 2.0;
        a = 0.0;
        c = std::atan2(r( 0, 1),  r( 1, 1));
    }
    else if(is_approx_equal(r(2, 0), 1.0))
    {
        b = -EIGEN_PI / 2.0;
        a = 0;
        c = -atan2(r( 0, 1),  r( 1, 1));
    }
    else
    {
        b = atan2(-r(2, 0), std::sqrt(r(0, 0) * r(0, 0) + r(1 ,0) * r(1 ,0)));
        a = atan2(r(1, 0), r(0, 0));
        c = atan2(r(2, 1), r(2, 2));
    }
    return Eigen::Vector3d{a, b, c};
}

    //defenition 3.7  page 77 modern robotics
Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d skew;
    skew <<
        0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return skew;
}
    //defenition 3.7  page 77 modern robotics
Eigen::Vector3d from_skew_symmetric(const Eigen::Matrix3d &m)
{
    return Eigen::Vector3d(m(2,1), m(0,2), m(1,0));
}
//definition 3.20 page 98
Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    Eigen::MatrixXd adj(6, 6);
    //define nullmatrix
    Eigen::Matrix3d nullmatrix = Eigen::Matrix3d::Zero();

    adj.block<3,3>(0,0) = r;
    adj.block<3,3>(0,3) = nullmatrix;
    adj.block<3,3>(3,0) = skew_symmetric(p)*r;
    adj.block<3,3>(3,3) = r;
    return adj;
}

    //definition 3.20 page 98
Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix4d &tf)
{
    Eigen::MatrixXd adt(6, 6);
    // Picks R & p from  T matrix
    Eigen::Matrix3d R = tf.block<3, 3>(0, 0);
    Eigen::Vector3d p = tf.block<3, 1>(0, 3);
    Eigen::Matrix3d nullmatrix = Eigen::Matrix3d::Zero();

    adt.block<3, 3>(0, 0) = R;
    adt.block<3, 3>(0, 3) = nullmatrix;
    adt.block<3, 3>(3, 0) = skew_symmetric(p) * R;
    adt.block<3, 3>(3, 3) = R;

    return adt;
}
    //definition 3.20 page 98
Eigen::VectorXd adjoint_map(const Eigen::VectorXd &twist, const Eigen::Matrix4d &tf)
{
    Eigen::VectorXd adj_map = adjoint_matrix(tf) * twist;
    return adj_map;
}

//eq 3.70 page 96
Eigen::VectorXd twist(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    Eigen::VectorXd twist(6);
    twist << w(0), w(1), w(2), v(0), v(1), v(2);
    return twist;
}

    //cahpter 3.3.2.2 page 101
Eigen::VectorXd twist(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h, double angular_velocity)
{
    Eigen::VectorXd twist(6);
    Eigen::Vector3d w = s * angular_velocity;
    Eigen::Vector3d v = -w.cross(q) + h * w;
    twist << w, v;
    return twist;
}

    //3.71 page 96
Eigen::Matrix4d twist_matrix(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    Eigen::Matrix4d twist = Eigen::Matrix4d::Zero();
    twist.block<3,3>(0, 0) = skew_symmetric(w);
    twist.block<3, 1>(0, 3) = v;
    return twist;
}

    //    3.71 page 96
Eigen::Matrix4d twist_matrix(const Eigen::VectorXd &twist)
{
    Eigen::Matrix4d twist_m = Eigen::Matrix4d::Zero();
    twist_m.block<3, 3>(0, 0) = skew_symmetric(twist.head<3>());
    twist_m.block<3, 1>(0, 3) = twist.tail<3>();
    return Eigen::Matrix4d::Identity();
}

    //defenition 3.24 page 102
Eigen::VectorXd screw_axis(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    Eigen::VectorXd screw (6);
    //kansje legge til en if variant her

    screw << w(0), w(1), w(2), v(0), v(1), v(2);

    return screw;
}

    // 3.5 summery page 110
Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{

    Eigen::VectorXd screw(6);
    Eigen::Vector3d v = -s.cross(q) + h * s;
    screw << s, v;
    return screw;

}
// 3.51 formula page 82 mr 2019
Eigen::Matrix3d matrix_exponential(const Eigen::Vector3d &w, double theta)
{
    Eigen::Matrix3d skew_w{skew_symmetric(w)};
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();


    return I + std::sin(theta) * skew_w + (1.0 - std::cos(theta)) * skew_w * skew_w;
}

    //proposition 3.25 page 103 MR 2019
Eigen::Matrix4d matrix_exponential(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Zero();

    if(is_approx_equal(w.norm(), 1.0))
    {
        const Eigen::Matrix3d skew_w{skew_symmetric(w)};
        const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        const Eigen::Matrix3d ROT_m = matrix_exponential(w, theta);

        Eigen::Matrix3d pt = I + std::sin(theta) * skew_w + (1.0 - std::cos(theta)) * skew_w * skew_w;

        Eigen::Vector3d p = pt * v;

        T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = ROT_m;
        T.block<3, 1>(0, 3) = p;
    }
    else if(is_approx_equal(v.norm(),1.0) && is_approx_equal(w.norm(),0.0))
    {
        T = Eigen::Matrix4d::Identity();
        T.block<3, 1>(0, 3) = v * theta;
    }
    return T;
}

    //page 103 MR 2019
Eigen::Matrix4d matrix_exponential(const Eigen::VectorXd &screw, double theta)
{
    return matrix_exponential(screw.head<3>(), screw.tail<3>(), theta);
}

    //Algorithm page 87 MR 2019
std::pair<Eigen::Vector3d, double> matrix_logarithm(const Eigen::Matrix3d &r)
{

    double theta = 0.0;
    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    if (is_approx_equal(r.trace(), -1.0))
    {
        theta = EIGEN_PI;
        if (!is_approx_equal(r(2, 2), -1.0))
        {
            w = 1.0 / (std::sqrt(2.0 * (1.0 + (r(2, 2))))) * Eigen::Vector3d(r(0, 2), r(1, 2), 1.0 + r(2, 2));
        }
        else if (!is_approx_equal(r(1, 1), -1.0))
        {
            w = 1.0 / (std::sqrt(2.0 * (1.0 + (r(2, 2))))) * Eigen::Vector3d(r(0, 1), 1.0 + r(1, 1), r(2, 1));
        }
        else if (!is_approx_equal(r(0, 0), -1.0))
        {
            w = 1.0 / (std::sqrt(2.0 * (1.0 + (r(2, 2))))) * Eigen::Vector3d(1.0 + r(0, 0), r(1, 0), r(2, 0));
        }
    }
    else if (!r.isIdentity())
    {
        theta = std::acos(0.5 * (r.trace() - 1.0));
        w = from_skew_symmetric(1.0 / (2.0 * std::sin(theta)) * (r - r.transpose()));
    }
    return std::make_pair(w, theta);
}

//algorithm fra s104 MR 2019 for next functions
Eigen::Matrix3d G_inv(const Eigen::Vector3d& w, const double& theta)
{
    Eigen::Matrix3d matrix_i = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d skew_w{skew_symmetric(w)};

    return matrix_i * 1.0 / theta - 0.5 * skew_w + (1 /theta - 0.5 * ((1.0/std::tan(theta))*(theta / 2.0))) * skew_w * skew_w;
}

    //algorithm fra s104 MR 2019
std::pair<Eigen::VectorXd, double> matrix_logarithm(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{

    double theta = 0.0;
    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    if (r.isIdentity())
    {
        v = p.normalized();
        theta = p.norm();
    }
    else
    {
        std::tie(w, theta) = matrix_logarithm(r);
        v = G_inv(w, theta) *p;
    }
    Eigen::VectorXd s(6);
    s << w, v;

    return std::make_pair(s, theta);
}
    //algorithm fra s104 MR 2019
std::pair<Eigen::VectorXd, double> matrix_logarithm(const Eigen::Matrix4d &tf)
{
    Eigen::Matrix3d R = tf.block<3, 3>(0, 0);
    Eigen::Vector3d p = tf.block<3, 1>(0, 3);

    double theta = 0.0;
    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    if (R.isIdentity())
    {
        v=p.normalized();
        theta = p.norm();
    }

    else
    {
        std::tie(w, theta) = matrix_logarithm(R);
        v = G_inv(w, theta) * p;
    }
    Eigen::VectorXd s(6);
    s << w, v;

    return std::make_pair(s, theta);
}

    //page 72 MR 2019
Eigen::Matrix3d rotate_x(double radians)
{
    Eigen::Matrix3d matrix;;
    matrix <<
        1.0, 0.0, 0.0,
        0.0, std::cos(radians), -std::sin(radians),
        0.0, std::sin(radians), std::cos(radians);
    return matrix;
}

    //page 72 MR 2019
Eigen::Matrix3d rotate_y(double radians)
{
    Eigen::Matrix3d matrix;
    matrix <<
        std::cos(radians), 0.0, std::sin(radians),
        0.0, 1, 0.0,
        -std::sin(radians), 0.0, std::cos(radians);
    return matrix;;
}

    //page 72 MR 2019
Eigen::Matrix3d rotate_z(double radians)
{
    Eigen::Matrix3d matrix;
    matrix <<
        std::cos(radians), -std::sin(radians), 0.0,
        std::sin(radians), std::cos(radians), 0.0,
        0.0, 0.0, 1;
    return matrix;
}
//eq 3.16 MR 2019 page 65
Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z)
{
    Eigen::Matrix3d matrix;
    matrix <<
        x, y, z;
    return matrix;
}

    //B.2 page 582
Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e)
{
    Eigen::Matrix3d matrix;
    Eigen::Matrix3d ROT_z = rotate_z(e(0)); //e(0)=alpha
    Eigen::Matrix3d ROT_y = rotate_y(e(1)); // e(1)=beta
    Eigen::Matrix3d ROT_x = rotate_x(e(2)); // e(2)=gamma
    matrix = ROT_z * ROT_y * ROT_x;
    return matrix;
}
    // 3.51 formula page 82 mr 2019
Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double radians)
{
    Eigen::Matrix3d skew = skew_symmetric(axis);
    Eigen::Matrix3d matrix_i = Eigen::Matrix3d::Identity();
    return matrix_i + std::sin(radians) * skew + (1 - std::cos(radians)) * skew * skew;
}
//pick ut rotation part from 4x4 transformation vector
Eigen::Matrix3d rotation_matrix(const Eigen::Matrix4d &tf)
{
    Eigen::Matrix3d rotation_matrix = tf.block<3, 3>(0, 0) ;
    return rotation_matrix;
}
//eq 3.62 page 87 MR 2019
Eigen::Matrix4d transformation_matrix(const Eigen::Vector3d &p)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 1>(0, 3) = p;
    return T;
}
    //eq 3.62 page 87 MR 2019
Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = r;
    return T;
}
    //eq 3.62 page 87 MR 2019
Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = r;
    T.block<3, 1>(0, 3) = p;
    return T;
}

}
