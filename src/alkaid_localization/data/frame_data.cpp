/*
 * @Description: definition for frame data
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-07
 */

#include "frame_data.hpp"

namespace data_ns
{

/**
 * @brief    get rotation from matrix T
 * @param none
 * @note
 **/
Quatf FrameData::GetRotation()
{
    Quatf q(_pose.block<3, 3>(0, 0));
    return q;
}

/**
 * @brief    get translation from matrix T
 * @param none
 * @note
 **/
Vec3f FrameData::GetTranslation()
{
    return _pose.block<3, 1>(0, 3);
}

/**
 * @brief    quaternions normalization
 * @param none
 * @note
 **/
void FrameData::QuatNorm()
{
    Quatf q(_pose.block<3, 3>(0, 0));
   q.normalize();
   _pose.block<3,3>(0,0) = q.toRotationMatrix();
}

}; // namespace data_ns