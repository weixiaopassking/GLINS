#ifndef _KEY_FRAME_HPP
#define _KEY_FRAME_HPP
#ifndef _FRAME_DATA_HPP
#define _FRAME_DATA_HPP

#include "geometry_data.hpp"
#include "cloud_data.hpp"

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
    Mat4d _pose = Mat4d::Identity();
    CloudData::CLOUD_PTR _cloud_ptr;
     double _time_stamp = 0.0;
    unsigned int _index = 0;
    FrameType _type = FrameType::NORMAL;

    Quatd GetRotation();
    Vec3f GetTranslation();
    void QuatNorm();
};
} // namespace data_ns

#endif //_FRAME_DATA_HPP