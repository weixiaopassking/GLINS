/*
 * @Description: 点云数据发布
  * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */

#ifndef CLOUD_PUBLISHER_HPP_
#define CLOUD_PUBLISHER_HPP_

//ros 
#include <ros/ros.h>
//c++
#include <string>
//自定义点云数据类型
#include "../sensor_data/cloud_data.hpp"

namespace multisensor_localization
{

  class CloudPublisher
  {
  public:
    CloudPublisher(ros::NodeHandle &nh,
                   std::string topic_name,
                   std::string frame_id,
                   size_t buff_size);
    CloudPublisher() = default;

    void Publish(CloudData::CLOUD_PTR &cloud_ptr_input, double time);
    void Publish(CloudData::CLOUD_PTR &cloud_ptr_input);

    bool HasSubscribers();

  private:
    void PublishData(CloudData::CLOUD_PTR &cloud_ptr_input,ros::Time time);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
  };


} // namespace multisensor_localization

#endif