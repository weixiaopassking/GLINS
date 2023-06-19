#ifndef _POSE_RECKON_HPP
#define _POSE_RECKON_HPP


#include "common/eigen_types.hpp"
#include "common/math_utils.hpp"
#include <iostream>
#include <unistd.h>

class pose_reckon
{
  public:
    pose_reckon();
    void init();
    void run();

  private:
    Vec3d v_body;  // 车体自身速度
    Vec3d w_body;  // 车体自身角速度
    Vec3d a_body;  // 车体自身加速度
    Se3d ego_pose; // body2world系统
    double dt;     // 积分时间
};

#endif //_POSE_RECKON_HPP