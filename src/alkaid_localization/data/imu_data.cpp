#include "imu_data.hpp"
namespace data_ns
{

data_ns::Mat3f IMUData::GetRotation()
{
   data_ns::Quatf  q(_orientation.w,_orientation.x,_orientation.y, _orientation.z);
    data_ns::Mat3f  matrix = q.matrix();
    return matrix;
}
} // namespace data_ns
