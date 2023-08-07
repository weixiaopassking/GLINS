/*
 * @Description: tools for debug
 * @Function: 
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 */

#ifndef _CLOUD_PUB_HPP
#define _CLOUD_PUB_HPP

//data
#include "../data/cloud_data.hpp"
//system
#include <ros/ros.h>
#include <string>

namespace pub_ns
{

class CloudPub
{
  public:
    CloudPub() = delete;//must be set nh
    CloudPub(ros::NodeHandle &nh,const  std::string & topic_name, const std::string& frame_id,const size_t buffer_size);

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