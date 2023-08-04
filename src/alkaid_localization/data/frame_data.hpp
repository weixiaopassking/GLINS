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
        KEY,     // constitute local map
        NORMAL,  // from evert scan
        INVALID, // no use
        HISTORICAL
    };

  public:
    data_ns::Mat4f _pose = data_ns::Mat4f::Identity();
    CloudData::CLOUD_PTR _cloud_ptr;
    double _time_stamp = 0.0;
    unsigned int _index = 0;
    FrameType _type = FrameType::NORMAL;

    Quatf GetRotation();
    Vec3f GetTranslation();
    void QuatNorm();
};
} // namespace data_ns

#endif //_FRAME_DATA_HPP