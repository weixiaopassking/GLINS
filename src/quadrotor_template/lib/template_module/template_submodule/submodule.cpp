/**
*****************************************************************************
*  Copyright (C), 2023-2026,robotics gang
*  @file    submodule.hpp
*  @brief  模块模板
*  @author  robotics gang
*  @date    2023/7/20
*  @version v0.1
*  @ref
****************************************************************************
*/

#include "./submodule.hpp"

bool SubModule::Function()
{
    return true;
}

std::ostream &operator<<(std::ostream &os, const SubModule &obj)
{
    os << "调试信息";
    return os;
}