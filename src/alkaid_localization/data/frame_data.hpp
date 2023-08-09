/*
 * @Description:definition for frame data
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-07
 */

#ifndef _FRAME_DATA_HPP
#define _FRAME_DATA_HPP

#include "cloud_data.hpp"
#include "geometry_data.hpp"

namespace data_ns
{

enum FrameType
{
    KEY,     // select from general frame
    NORMAL,  // general frame
    INVALID, // invalid frame
    RESERVED
};
struct FrameData
{
    FrameData(const double &time_stamp = 0.0, const data_ns::Mat4f &pose = data_ns::Mat4f::Identity(),
              const data_ns::Vec3f &linear_velocity = data_ns::Vec3f::Zero(),
              const data_ns::Vec3f &accel_bais = data_ns::Vec3f::Zero(),
              const data_ns::Vec3f &gyro_bais = data_ns::Vec3f::Zero(), FrameType type = FrameType::NORMAL)
    {
        _time_stamp = time_stamp;
        _pose = pose;
        _linear_velocity = linear_velocity;
        _gyro_bais = gyro_bais;
        _accel_bais = accel_bais;
        _type=type;
    }

    // values
    data_ns::Mat4f _pose = data_ns::Mat4f::Identity();
    data_ns::Vec3f _linear_velocity = data_ns::Vec3f::Zero();
    data_ns::Vec3f _gyro_bais = data_ns::Vec3f::Zero();
    data_ns::Vec3f _accel_bais = data_ns::Vec3f::Zero();
    double _time_stamp = 0.0;
    CloudData::CLOUD_PTR _cloud_ptr;
    unsigned int _index = 0;
    FrameType _type = FrameType::NORMAL;

    // fcuntion
    data_ns::Quatf GetQuaternion();
    data_ns::Mat3f GetRotation();
    data_ns::Vec3f GetTranslation();
    void QuatNorm();

}; // class FrameData
} // namespace data_ns

#endif //_FRAME_DATA_HPP