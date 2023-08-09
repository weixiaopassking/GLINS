/*
 * @Description:imu pre integration
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 */

#ifndef _IMU_INTERGRATION_HPP
#define _IMU_INTERGRATION_HPP

// data
#include "../../data/frame_data.hpp"
#include "../../data/geometry_data.hpp"

namespace module_ns
{
class IMUIntegration
{
  public:
    struct Options
    {
        data_ns::Vec3f gyro_bais = data_ns::Vec3f::Zero();  // set zero init
        data_ns::Vec3f aceel_bais = data_ns::Vec3f::Zero(); // set zero init

        double gyro_noise = 1e-2;  // use standard
        double accel_noise = 1e-2; // use standard
    };

    IMUIntegration();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen bytes align

        ~IMUIntegration();

  private:
    Options _option; // param config

}; // class IMUIntegration
} // namespace module_ns

#endif //_IMU_INTERGRATION_HPP