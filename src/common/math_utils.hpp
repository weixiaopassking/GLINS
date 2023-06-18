/**
*****************************************************************************
*  Copyright (C), 2022-2024, gang
*  @file    pipeline.cpp
*  @brief 数学工具
*  @author  robotics gang
*  @date    2023/6/18
*  @version 0.1
*  @ref  none
****************************************************************************
*/

#ifndef _MATH_HPP
#define _MATH_HPP

#include <math.h>

namespace math
{
constexpr double kDEG2RAD = M_PI / 180.0; // 弧度制转角度制
constexpr double kRAD2DEG = 180.0 / M_PI;
constexpr double gravity = 9.81; // 重力 m/s^2
} // namespace math

#endif //_MATH_HPP