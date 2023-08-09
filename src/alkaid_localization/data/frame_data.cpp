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
FrameData::FrameData(const double time_stamp, const data_ns::Mat4f &pose, const data_ns::Vec3f &linear_velocity,
                     const data_ns::Vec3f &accel_bais, const data_ns::Vec3f &gyro_bais, const FrameType type)
{
    _time_stamp = time_stamp;
    _pose = pose;
    _linear_velocity = linear_velocity;
    _gyro_bais = gyro_bais;
    _accel_bais = accel_bais;
    _type = type;
}
/**
 * @brief    get rotation from matrix T
 * @param none
 * @note
 **/
Quatf FrameData::GetQuaternion()const
{
    Quatf q(_pose.block<3, 3>(0, 0));
    return q;
}

/**
 * @brief    get rotation from matrix T
 * @param none
 * @note
 **/
data_ns::Mat3f FrameData::GetRotation()const
{
    return _pose.block<3, 3>(0, 0);
}

/**
 * @brief    get translation from matrix T
 * @param none
 * @note
 **/
Vec3f FrameData::GetTranslation()const
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
    _pose.block<3, 3>(0, 0) = q.toRotationMatrix();
}



}; // namespace data_ns