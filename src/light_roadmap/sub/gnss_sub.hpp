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

namespace sub
{
class GnssSub : public SubBase
{
  public:
    GnssSub();
    ~GnssSub();
    void ParseData();
private:
void MsgCallback();
bool HasSubscribed();

private:
ros::NodeHandle _nh;
};
} // namespace sub

#endif //_GNSS_SUB_HPP