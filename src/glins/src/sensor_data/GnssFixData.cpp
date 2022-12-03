/*
 * @Description: gnss fix data type define
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-22
 * @Note: Using the WGS 84 reference ellipsoid
 */

// relevent
#include "../../include/sensor_data/GnssDataType.hpp"
// thirdpart lib
#include <glog/logging.h>

namespace glins
{
    /*------------------------------GnssFixData--------------------------------*/
    /*static value definition*/
    bool GnssFixData::flag_origin_fixed = false;
    GeographicLib::LocalCartesian GnssFixData::geo_converter;

    /**
     * @brief  fix origin  under ENU coordinates
     * @note
     * @todo
     **/
    void GnssFixData::FixOrigin()
    {
        geo_converter.Reset(latitude, longtitude, altitude);
        flag_origin_fixed = true;
    }

    /**
     * @brief update xyz axis odom under   ENU coordinates
     * @note
     * @todo
     **/
    void GnssFixData::UpdateGnssOdom()
    {
        if (flag_origin_fixed == false)
        {
            LOG(ERROR) << "Gnss fix has not been fixed " << std::endl;
        }
        /*update xyz axis odom*/
        geo_converter.Forward(latitude, longtitude, altitude, local_east, local_north, local_up);
    }

    /*------------------------------GnssRawData--------------------------------*/

} // namespace glins