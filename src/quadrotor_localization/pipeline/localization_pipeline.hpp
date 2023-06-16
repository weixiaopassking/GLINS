
/**
*****************************************************************************
*  Copyright (C), 2022-2024,robotics gang
*  @file    pipeline.hpp
*  @brief 定位调度
*  @author  robotics gang
*  @date    2023/6/16
*  @version 0.1
*****************************************************************************
*/

#include <iostream>

class localization_pipeline
{

  public:
    localization_pipeline();

    bool init();

    bool run();

    ~localization_pipeline();
};