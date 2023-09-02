/*
 * @Description:imu pre integration
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 * @refer github:gaoxiang
 */

#include "imu_preintergration.hpp"

namespace module_ns
{
IMUPreIntegration::IMUPreIntegration()
{
    _gyro_bais = _option.gyro_bais_init;
    _aceel_bais = _option.accel_bais_init;
    _noise_gryo_accel.diagonal() << pow(_option.gyro_noise, 2), pow(_option.gyro_noise, 2), pow(_option.gyro_noise, 2),
        pow(_option.accel_noise, 2), pow(_option.accel_noise, 2), pow(_option.accel_noise, 2);
}

void IMUPreIntegration::UpdateIMUData(data_ns::IMUData imu_data, const double dt)
{
    /*1--measurment variable (has  minus the bais) */
    data_ns::Vec3f accel = imu_data._accel - _aceel_bais;
    data_ns::Vec3f gyro = imu_data._gyro - _gyro_bais;
    /*2--update dv and dp*/
    _dp = _dp + _dv * dt + 0.5 * _dR.matrix() * accel * dt * dt;
    _dv = _dv + _dR * accel * dt;
    /*3--noise recursion matrix of coefficients  */
    Eigen::Matrix<float, 9, 9> A;
    A.setIdentity();
    Eigen::Matrix<float, 9, 6> B;
    B.setZero();

    data_ns::Mat3f accel_hat = data_ns::SO3f::hat(accel);
    A.block<3, 3>(3, 0) = (-1) * _dR.matrix() * dt * accel_hat;
    A.block<3, 3>(6, 0) = (-0.5) * _dR.matrix() * accel_hat * dt * dt;
    A.block<3, 3>(6, 3) = dt * data_ns::Mat3f::Identity();

    B.block<3, 3>(3, 3) = _dR.matrix() * dt;
    B.block<3, 3>(6, 3) = 0.5f * _dR.matrix() * dt * dt;

    /*4--update jacobi xxx round bais(accel gyro)*/
    _dp_round_accel_bais = _dp_round_accel_bais + _dv_round_accel_bais * dt - 0.5 * _dR.matrix() * dt * dt;
    _dp_round_gyro_bais = _dp_round_gyro_bais + _dv_round_accel_bais * dt -
                          0.5 * _dR.matrix() * accel_hat * _dR_round_gyro_bais * dt * dt;
    _dv_round_accel_bais = _dv_round_accel_bais - _dR.matrix() * dt;
    _dv_round_gyro_bais = _dv_round_gyro_bais - _dR.matrix() * dt * accel_hat * _dR_round_gyro_bais;

    data_ns::Mat3f Jr = data_ns::SO3f::jr(gyro * dt);
    data_ns::SO3f deltaR = data_ns::SO3f::exp(gyro * dt);
    _dR = _dR * deltaR;

    A.block<3, 3>(0, 0) = deltaR.matrix().transpose();
    B.block<3, 3>(0, 0) = Jr * dt;

    /*5--update noise、dR/bg、dt increasment*/
    _cov = A * _cov * A.transpose() + B * _noise_gryo_accel * B.transpose();
    _dR_round_gyro_bais = deltaR.matrix().transpose() * _dR_round_gyro_bais - Jr * dt;
    _dt += dt;
}

data_ns::FrameData IMUPreIntegration::UpdateState(const data_ns::FrameData start_state, const data_ns::Vec3f &gravity)
{
    data_ns::Mat3f Rj = start_state._pose.block<3, 3>(0, 0) * _dR.matrix();
    data_ns::Vec3f vj = start_state._pose.block<3, 3>(0, 0) * _dv + start_state._linear_velocity + gravity * _dt;
    data_ns::Vec3f pj = start_state._pose.block<3, 3>(0, 0) * _dp + start_state.GetTranslation() +
                        start_state._linear_velocity * _dt + 0.5 * gravity * _dt * _dt;
    data_ns::Mat4f transform_matrix = data_ns::Mat4f::Identity();
    transform_matrix.block<3, 3>(0, 0) = Rj;
    transform_matrix.block<3, 1>(0, 3) = pj;

    data_ns::FrameData res_state = data_ns::FrameData(start_state._time_stamp + _dt, transform_matrix, pj, vj);
    res_state._gyro_bais = _gyro_bais;
    res_state._accel_bais = _aceel_bais;
    return res_state;
}

IMUPreIntegration::~IMUPreIntegration()
{
}

} // namespace module_ns
