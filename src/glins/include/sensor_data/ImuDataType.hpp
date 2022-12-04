/*
 * @Description: 9-axis imu  data type define
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-18
 * @Todo:
 */

#ifndef IMU_DATA_TYPE_HPP_
#define IMU_DATA_TYPE_HPP_

// c++ lib
#include <cmath>
// thirdpart lib
#include <Eigen/Dense>

namespace glins
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

        struct MagneticComponent
        {
            double x = 0.0, y = 0.0, z = 0.0;
        };

        class Orientation
        {
        public:
            double x = 0.0, y = 0.0, z = 0.0, w = 0.0;
            void Normlize()
            {
                double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
                x /= norm;
                y /= norm;
                z /= norm;
                w /= norm;
            }
        };
        double time_stamp = 0.0;
        LinearAcceleration linear_acceleration;
        AngularVelocity angular_velocity;
        MagneticComponent magnetic_component;
        Orientation orientation;

    public:
        Eigen::Matrix3f OrientationToRotationMatrix();
    }; // ImuData

} // namespace glins

#endif