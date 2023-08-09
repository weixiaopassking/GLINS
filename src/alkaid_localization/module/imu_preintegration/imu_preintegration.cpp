/*
 * @Description:imu pre integration
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 * @refer
 */

#include "imu_preintergration.hpp"

namespace module_ns
{
IMUPreIntegration::IMUPreIntegration()
{
}

void IMUPreIntegration::UpdateIMUData(data_ns::IMUData imu_data)
{ 
    /*1--minus the bais */
    data_ns::Vec3f angular_velocity = imu_data._accel - _aceel_bais;
    data_ns::Vec3f linear_acceleration = imu_data._gyro - _gyro_bais;
    /*2--*/
    

}

data_ns::FrameData IMUPreIntegration::UpdateStates(const data_ns::FrameData start_state, const data_ns::Vec3f &gravity)
{
}

IMUPreIntegration::~IMUPreIntegration()
{
}

} // namespace module_ns
