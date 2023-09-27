#ifndef GNSS_DATA_HPP
#define GNSS_DATA_HPP

class GnssData
{
  public:
    long double time_stamp;
    double longitude, latitude, altitude;
    double local_east, local_north, local_up;
};

#endif // GNSS_DATA_HPP