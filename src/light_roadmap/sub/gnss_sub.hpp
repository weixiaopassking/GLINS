/*
 * @Description: gnss subscriber
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-09-24
 */

#ifndef _GNSS_SUB_HPP
#define _GNSS_SUB_HPP

#include "sub_base.hpp"
#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

class GnssSub : public SubBase
{
  public:
    GnssSub();
    void ParseData();
    void MsgCallback();
    bool HasSubscribed();
    ~GnssSub();

  private:
    ros::NodeHandle _nh;
};

#endif //_GNSS_SUB_HPP