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

#ifndef _SUBMODULE_HPP
#define _SUBMODULE_HPP
#include <iostream>

class SubModule
{

  public:
    SubModule() = delete;
    bool Function();
    friend std::ostream &operator<<(std::ostream o, const SubModule &obj);

};

#endif //_SUBMODULE_HPP