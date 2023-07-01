/**
*****************************************************************************
*  @file    pipeline.cpp
*  @brief 矩阵运算头文件
*  @author  robotics gang
*  @date    2023/6/18
*  @version 0.1
*  @ref https://blog.csdn.net/caiqidong321 Sophus库安装和使用
****************************************************************************
*/

#include "sophus_use.hpp"
#include "sophus/se3.hpp"
#include <Eigen/Dense>

sophus_use::sophus_use()
{
    this->demo1();
}

void sophus_use::demo1()
{
    /*1.旋转矩阵R <-->李群SO3*/
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix(); // 绕Z轴旋转90度
    Sophus::SO3d SO3_R(R);
    std::cout << SO3_R.matrix() << std::endl;

    /*2.四元数<-->李群SO3*/
    Eigen::Quaterniond q(R);
    Sophus::SO3d SO3_q(q);
    std::cout << SO3_q.matrix() << std::endl;

    /*3.李群SO3<-->李代数so3*/
    Eigen::Vector3d so3 = SO3_R.log();
    std::cout << so3.transpose() << std::endl;
    Sophus::SO3 SO3_so3 = Sophus::SO3d::exp(so3);

    /*4.李代数<-->反对称矩阵*/
    // Eigen::Matrix3d R_v = Sophus::SO3d::hat(so3);
    // std::cout << R_v << std::endl;
    // Eigen::Vector3d so3_Rv 
}

void sophus_use::demo2()
{
}