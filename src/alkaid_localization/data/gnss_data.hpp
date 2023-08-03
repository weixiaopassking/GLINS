#ifndef _GNSS_DATA_HPP_
#define _GNSS_DATA_HPP_

#include <GeographicLib/LocalCartesian.hpp>
#include <deque>
#include <iostream>

/*
unorder  ENU
*/

namespace data_ns
{
class GNSSData
{
  public:
    double _time_stamp = 0.0;
    double _longitude = 0.0, _latitude= 0.0, _altitude = 0.0;
    double _local_east = 0.0, _local_north = 0.0, _local_up = 0.0;
    int _status = -1; // default no fix
    int _service=1;//default gps
    // todo use
    // float _pseudorange;//raw data;
    // float  _doppler_shift;

  private:
    static GeographicLib::LocalCartesian _geo_converter;
    static bool _origin_position_inited;

  public:
    void Init();
    void Update();
}; // class GNSSData

} // namespace data_ns

#endif