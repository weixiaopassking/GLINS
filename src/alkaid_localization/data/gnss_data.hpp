/*
 * @Description: definition for gnss data
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-07
 * @Note:
 * under ENU coordinate
 * A  fix is valid when status >= STATUS_FIX.
 * https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html
 */

#ifndef _GNSS_DATA_HPP_
#define _GNSS_DATA_HPP_

#include <GeographicLib/LocalCartesian.hpp>
#include <deque>
#include <iostream>

namespace data_ns
{
class GNSSData
{
  public:
    double _time_stamp = 0.0; // sec use unix time
    double _longitude = 0.0, _latitude= 0.0, _altitude = 0.0;
    double _local_east = 0.0, _local_north = 0.0, _local_up = 0.0;
    int _status = -1; // default no fix
    int _service=1;//default gps

  private:
    static GeographicLib::LocalCartesian _geo_converter;
    static bool _origin_position_inited;

  public:
    void Init();
    void Update();
}; // class GNSSData

} // namespace data_ns

#endif