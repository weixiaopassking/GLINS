#include "frame_data.hpp"

namespace data_ns
{

Quatd FrameData::GetRotation()
{
    Quatd q(_pose.block<3, 3>(0, 0));
    return q;
}

Vec3d FrameData::GetTranslation()
{
    return _pose.block<3, 1>(0,3);
}
void FrameData::QuatNorm()
{
    Quatd q(_pose.block<3, 3>(0, 0));
    _pose=q.normalize().toRotationMatrix();
}

}; // namespace data_ns