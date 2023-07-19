#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CloudFilterInterface
{
  public:
    CloudFilterInterface() = delete;
    virtual bool Filter() = 0;
};