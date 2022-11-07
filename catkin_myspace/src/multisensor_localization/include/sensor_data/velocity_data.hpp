/*
 * @Description: 速度数据存放
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-14
 * @Todo
 */

#ifndef VELOCITY_DATA_HPP_
#define VELOCITY_DATA_HPP_

#include <deque>
#include <Eigen/Dense>

namespace multisensor_localization
{
  class VelocityData
  {
  public:
    struct LinearVelocity
    {
      double x = 0.0, y = 0.0, z = 0.0;
    };

    struct AngularVelocity
    {
      double x = 0.0, y = 0.0, z = 0.0;
    };

    double time_stamp_ = 0.0;
    LinearVelocity linear_velocity_;
    AngularVelocity angular_velocity_;

  public:
    void TransformCoordinate(Eigen::Matrix4f transform_matrix);
    static bool SyncData(std::deque<VelocityData> &unsynced_data_buff,
                         std::deque<VelocityData> &synced_data_buff,
                         double sync_time);
  };

  
}
#endif