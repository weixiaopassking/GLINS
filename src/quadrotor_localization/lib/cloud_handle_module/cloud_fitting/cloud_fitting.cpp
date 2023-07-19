/**
*****************************************************************************
*  Copyright (C), 2023-2026,robotics gang
*  @file    cloud_io.hpp
*  @brief  点云的读写
*  @author  robotics gang
*  @date    2023/7/18
*  @version v0.1
*  @ref https://github.com/gaoxiang12
****************************************************************************
*/

#include "cloud_fitting.hpp"

/**
 * @brief 点云拟合平面(解析解)
 * @param points 观测点云
 * @param plane_coeffs 平面系数
 * @param eps 误差
 * @return bool
 * @note 静态成员函数
 */
bool CloudFitting::PlaneFitting(std::vector<Eigen::Vector3d> &points, Eigen::Matrix<double, 4, 1> &plane_coeffs,
                                const double eps)
{
    /*1--异常校验*/
    if (points.size() < 3)
    {
        return false;
    }
    /*2--设置A矩阵*/
    Eigen::MatrixXd A(points.size(), 4);
    for (int i = 0; i < points.size(); i++)
    {
        A.row(i).head<3>() = points[i].transpose();
        A.row(i)[3] = 1;
    }
    /*3--SVD分解*/
    Eigen::JacobiSVD svd(A, Eigen::ComputeThinV);
    plane_coeffs = svd.matrixV().col(3);

    /*4--误差校验*/
    for (int i = 0; i < points.size(); i++)
    {
        double error = plane_coeffs.head<3>().dot(points[i]) + plane_coeffs[3];
        if (error * error > eps)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 点云拟合直线(解析解)
 * @param points 观测点云
 * @param start_point 直线方程起点
 * @param direction 直线方程的方向向量
 * @return bool
 * @note 静态成员函数
 */
bool CloudFitting::LineFitting(std::vector<Eigen::Vector3d> &points, Eigen::Matrix<double, 3, 1> &start_point,
                               Eigen::Matrix<double, 3, 1> &direction, const double eps)
{
    /*1--异常校验*/
    if (points.size() < 2)
        return false;
    /*2--计算输入点云的均匀值p*/
    start_point =
        std::accumulate(points.begin(), points.end(), Eigen::Matrix<double, 3, 1>::Zero().eval()) / points.size();
    /*3--计算xk-p=yk^T*/
    Eigen::MatrixXd Y(points.size(), 3);
    for (int i = 0; i < points.size(); i++)
    {
        Y.row(i) = (points[i] - start_point).transpose();
    }
    /*5--svd分解*/
    Eigen::JacobiSVD svd(Y, Eigen::ComputeFullV); // 雅可比分解
    direction = svd.matrixV().col(0);             // 取第一列即最大

    /*6--检查eps*/
    for (const auto &p : points)
    {
        if (direction.cross(p - start_point).squaredNorm() > eps)
        {
            return false;
        }
    }
    return true;
}
