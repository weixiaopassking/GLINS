
#ifndef _CLOUD_PUB_HPP
#define _CLOUD_PUB_HPP

#include "../data/cloud_data.hpp"
#include <ros/ros.h>
#include <string>

namespace pub_ns
{

class CloudPub
{
  public:
    CloudPub() = default;
    CloudPub(ros::NodeHandle &nh, std::string topic_name, std::string frame_id, size_t buff_size);

    void Pub(data_ns::CloudData::CLOUD_PTR &cloud_ptr);
    void Pub(data_ns::CloudData::CLOUD_PTR &cloud_ptr, double time); // overload

    bool HasSubscribered();

  private:
    void PubData(data_ns::CloudData::CLOUD_PTR &cloud_ptr, ros::Time time);

  private:
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    std::string _frame_id;
};

} // namespace pub_ns

#endif //_CLOUD_PUB_HPP