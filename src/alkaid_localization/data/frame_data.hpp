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
class FrameData
{
    public:

    FrameData(const double time_stamp = 0.0, const data_ns::Mat4f &pose = data_ns::Mat4f::Identity(),
                                 const data_ns::Vec3f &linear_velocity = data_ns::Vec3f::Zero(),
                                 const data_ns::Vec3f &accel_bais = data_ns::Vec3f::Zero(),
                                 const data_ns::Vec3f &gyro_bais = data_ns::Vec3f::Zero(),
                                 const FrameType type = FrameType::NORMAL);


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
    data_ns::Quatf GetQuaternion() const ;
    data_ns::Mat3f GetRotation() const ;
    data_ns::Vec3f GetTranslation()const ;

    void QuatNorm();

}; // class FrameData
} // namespace data_ns

#endif //_FRAME_DATA_HPP