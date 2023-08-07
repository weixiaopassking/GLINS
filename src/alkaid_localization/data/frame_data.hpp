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
class FrameData
{
    enum FrameType
    {
        KEY,     // select from general frame
        NORMAL,  // general frame
        INVALID, // invalid frame
        RESERVED
    };

  public:
    data_ns::Mat4f _pose = data_ns::Mat4f::Identity();
    CloudData::CLOUD_PTR _cloud_ptr;
    double _time_stamp = 0.0;
    unsigned int _index = 0;
    FrameType _type = FrameType::NORMAL;

    data_ns::Quatf GetQuaternion();
    data_ns::Mat3f GetRotation();
    data_ns::Vec3f GetTranslation();
    void QuatNorm();
}; // class FrameData
} // namespace data_ns

#endif //_FRAME_DATA_HPP