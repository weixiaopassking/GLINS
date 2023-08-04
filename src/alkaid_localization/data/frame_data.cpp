#include "frame_data.hpp"

namespace data_ns
{

Quatf FrameData::GetRotation()
{
    Quatf q(_pose.block<3, 3>(0, 0));
    return q;
}

Vec3f FrameData::GetTranslation()
{
    return _pose.block<3, 1>(0, 3);
}
void FrameData::QuatNorm()
{
    Quatf q(_pose.block<3, 3>(0, 0));
   q.normalize();
   _pose.block<3,3>(0,0) = q.toRotationMatrix();
}

}; // namespace data_ns