/*
 * @Description: publisher point cloud
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-12-05
 * @Note: 
 */
#ifndef CLOUD_PUBLISHER_HPP_
#define CLOUD_PUBLISHER_HPP_

//ros 
#include <ros/ros.h>
//c++
#include <string>
//sensor data type
#include "../sensor_data/CloudDataType.hpp"

namespace glins
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