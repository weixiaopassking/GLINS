#include "gnss_sub.hpp"

namespace sub_ns
{

GNSSSub::GNSSSub(ros::NodeHandle &nh, const std::string topic_name, const size_t buffer_size)
{
    _nh = nh;
    _sub = _nh.subscribe(topic_name, buffer_size, &GNSSSub::MsgCallback, this);
}

void GNSSSub::MsgCallback(const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr)
{
    data_ns::GNSSData gnss_data;
    gnss_data._time_stamp = nav_sat_fix_ptr->header.stamp.toSec();
    gnss_data._latitude = nav_sat_fix_ptr->latitude;
    gnss_data._longitude = nav_sat_fix_ptr->longitude;
    gnss_data._altitude = nav_sat_fix_ptr->altitude;
    // todo for raw data
    //  gnss_data._status = nav_sat_fix_ptr->status.status;
    // gnss_data._service_ = nav_sat_fix_ptr->status.service;

    _gnss_data_deq.push_back(gnss_data);
}

void GNSSSub::ParseData(std::deque<data_ns::GNSSData> &gnss_data_deq)
{
    if (_gnss_data_deq.size() > 0)
    {
        gnss_data_deq.insert(gnss_data_deq.end(), _gnss_data_deq.begin(), _gnss_data_deq.end());
        _gnss_data_deq.clear();
    }
}
} // namespace sub_ns