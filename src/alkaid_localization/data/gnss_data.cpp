/*
 * @Description: definition for gnss data
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-07
 */

#include "gnss_data.hpp"
#include "../tools/tools.hpp"

namespace data_ns
{

bool GNSSData::_origin_position_inited = false;
GeographicLib::LocalCartesian GNSSData::_geo_converter;

/**
 * @brief gnss fix origin position
 * @param none
 * @note
 **/
void GNSSData::Init()
{
    _geo_converter.Reset(_latitude, _longitude, _altitude);
    _origin_position_inited = true;
}

/**
 * @brief update gnss odom
 * @param none
 * @note
 **/
void GNSSData::Update()
{
    if (_origin_position_inited == true)
    {
        _geo_converter.Forward(_latitude, _longitude, _altitude, _local_east, _local_north, _local_up);
        tools_ns::StatusAssert("GNSS origin has been set", __FUNCTION__);
    }
    else
    {
        tools_ns::ErrorAssert("GNSS origin has't been set", __FILE__, __FUNCTION__, __LINE__);
    }
}

} // namespace data_ns