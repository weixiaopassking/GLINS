
#include "gnss_data.hpp"

namespace data_ns
{

bool GNSSData::_origin_position_inited = false;
GeographicLib::LocalCartesian GNSSData::_geo_converter;

void GNSSData::Init()
{
    _geo_converter.Reset(_latitude, _longitude, _altitude);
    _origin_position_inited = true;
}

void GNSSData::Update()
{
    if (_origin_position_inited == true)
    {
        _geo_converter.Forward(_latitude, _longitude, _altitude, _local_east, _local_north, _local_up);
    }
    else
    {
        std::cout << "gnss has not been set origin" << std::endl;
        exit(-1);
    }
}

} // namespace data_ns