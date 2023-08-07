/*
 * @Description: gnss subscriber
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 */

#include "gnss_sub.hpp"

namespace sub_ns
{

/**
 * @brief    gnss sub init
 * @param 
 * @note
 **/
GNSSSub::GNSSSub(ros::NodeHandle &nh, const std::string & topic_name, const size_t buffer_size)
{
    _nh = nh;
    _sub = _nh.subscribe(topic_name, buffer_size, &GNSSSub::MsgCallback, this);
}

/**
 * @brief    gnss sub call back
 * @param
 * @note
 **/
void GNSSSub::MsgCallback(const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr)
{
    data_ns::GNSSData gnss_data;
    gnss_data._time_stamp = nav_sat_fix_ptr->header.stamp.toSec();
    gnss_data._latitude = nav_sat_fix_ptr->latitude;
    gnss_data._longitude = nav_sat_fix_ptr->longitude;
    gnss_data._altitude = nav_sat_fix_ptr->altitude;
    gnss_data._status = nav_sat_fix_ptr->status.status;
    gnss_data._service= nav_sat_fix_ptr->status.service;
    // todo for raw data

    _gnss_data_deq.push_back(gnss_data);
}

/**
 * @brief    parese data from gnss buffer
 * @param
 * @note
 **/
void GNSSSub::ParseData(std::deque<data_ns::GNSSData> &gnss_data_deq)
{
    if (_gnss_data_deq.size() > 0)
    {
        gnss_data_deq.insert(gnss_data_deq.end(), _gnss_data_deq.begin(), _gnss_data_deq.end());
        _gnss_data_deq.clear();
    }
}
} // namespace sub_ns