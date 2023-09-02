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
#include "../../data/imu_data.hpp"

namespace module_ns
{
class IMUPreIntegration
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen bytes align

        public :
        
        struct Options
    {
        data_ns::Vec3f gyro_bais_init = data_ns::Vec3f::Zero();
        data_ns::Vec3f accel_bais_init = data_ns::Vec3f::Zero();
        double gyro_noise =0; //1e-2;  // use standard
        double accel_noise = 0;//1e-2; // use standard
    };

    IMUPreIntegration();
    void UpdateIMUData(data_ns::IMUData imu_data, const double dt);
    data_ns::FrameData UpdateState(const data_ns::FrameData start_state, const data_ns::Vec3f &gravity);

    ~IMUPreIntegration();

  private:
    Options _option;                                     // param config
    data_ns::Vec3f _gyro_bais = data_ns::Vec3f::Zero();  // set zero init
    data_ns::Vec3f _aceel_bais = data_ns::Vec3f::Zero(); // set zero init
    data_ns::Vec3f _dv = data_ns::Vec3f::Zero();         // preintegrate
    data_ns::Vec3f _dp = data_ns::Vec3f::Zero();
    data_ns::SO3f _dR;
    data_ns::Mat9f _cov = data_ns::Mat9f::Zero(); // accumulated  noise matrix
    data_ns::Mat6f _noise_gryo_accel = data_ns::Mat6f::Zero();

    double _dt = 0;

    // Jacobi
    data_ns::Mat3f _dR_round_gyro_bais = data_ns::Mat3f::Zero();
    data_ns::Mat3f _dv_round_gyro_bais = data_ns::Mat3f::Zero();
    data_ns::Mat3f _dv_round_accel_bais = data_ns::Mat3f::Zero();
    data_ns::Mat3f _dp_round_gyro_bais = data_ns::Mat3f::Zero();
    data_ns::Mat3f _dp_round_accel_bais = data_ns::Mat3f::Zero();

}; // class IMUPreIntegration
} // namespace module_ns

#endif //_IMU_INTERGRATION_HPP