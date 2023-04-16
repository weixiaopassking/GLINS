/*
 * @Description: imu数据封装
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-14
 * @Todo xyz下划线规范??
 */

#ifndef IMU_DATA_HPP_
#define IMU_DATA_HPP_

#include <deque>
#include <cmath>
#include <Eigen/Dense>

namespace multisensor_localization
{
  class ImuData
  {
  public:
    struct LinearAcceleration
    {
      double x = 0.0, y = 0.0, z = 0.0;
    };

    struct AngularVelocity
    {
      double x = 0.0, y = 0.0, z = 0.0;
    };

    class Orientation
    {
    public:
      double x = 0.0, y = 0.0, z = 0.0, w = 0.0;

    public:
      void Normlize()
      {
        double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
        x /= norm;
        y /= norm;
        z /= norm;
        w /= norm;
      }
    };

    double time_stamp_ = 0.0;
    LinearAcceleration linear_acceleration_;
    AngularVelocity angular_velocity_;
    Orientation orientation_;

  public:
    Eigen::Matrix3f OrientationToMatrix();
    static bool SyncData(std::deque<ImuData> &unsynced_data_buff,
                         std::deque<ImuData> &synced_data_buff,
                         double sync_time);
  };


}//namespace multisensor_localization

#endif