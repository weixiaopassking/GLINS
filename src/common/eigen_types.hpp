/**
*****************************************************************************
*  Copyright (C), 2022-2024, gang
*  @file    pipeline.cpp
*  @brief 矩阵运算头文件
*  @author  robotics gang
*  @date    2023/6/18
*  @version 0.1
*  @ref  none
****************************************************************************
*/
#ifndef _EIGEN_TYPES_HPP
#define _EIGEN_TYPES_HPP

#include "sophus/se3.hpp"
#include <Eigen/Core>

using Vec3d = Eigen::Vector3d;
using Se3d = Sophus::SE3d;
using So3 = Sophus::SO3d;
#endif //_EIGEN_TYPES_HPP