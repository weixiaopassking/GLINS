#include "frame_data.hpp"

namespace data_ns
{

Quatd FrameData::GetRotation()
{
}
Vec3f FrameData::GetTranslation()
{
}
void FrameData::QuatNorm()
{
    Quatd q(_pose.block<3, 3>(0, 0));
    _pose=q.normalize().toRotationMatrix();
}

}; // namespace data_ns