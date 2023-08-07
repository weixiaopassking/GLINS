/*
 * @Description: definition for imu data
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-07
 */

#ifndef _IMU_DATA_HPP
#define _IMU_DATA_HPP
#include <cmath>
#include "geometry_data.hpp"

namespace data_ns
{
class IMUData
{
  public:
    struct LinearAcceleration
    {
        float x = 0.0, y = 0.0, z = 0.0;
    };

    struct AngularVelocity
    {
        float x = 0.0, y = 0.0, z = 0.0;
    };
    class Orientation
    {
      public:
        double x = 0.0, y = 0.0, z = 0.0, w = 0.0;

      public:
        void QuatNorm()
        {
            float norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
            x /= norm;
            y /= norm;
            z /= norm;
            w /= norm;
        }
    };

    data_ns::Mat3f   GetRotation();
    double _time_stamp = 0.0;
    LinearAcceleration _linear_acceleration;
    AngularVelocity _angular_velocity;
    Orientation _orientation;

}; // class IMUData
} // namespace data_ns

#endif //_IMU_DATA_HPP