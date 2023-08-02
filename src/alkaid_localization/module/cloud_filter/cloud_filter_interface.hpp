#ifndef _FILTER_INTERFACE_HPP
#define _FILTER_INTERFACE_HPP

#include "../../data/cloud_data.hpp"

namespace module_ns
{
class CloudFilterInterface
{
  public:
    CloudFilterInterface() = default;
    virtual bool Filter(const data_ns::CloudData::CLOUD_PTR &cloud_ptr,
                        data_ns::CloudData::CLOUD_PTR &filtered_cloud_ptr) = 0;
    virtual ~CloudFilterInterface() = default;
};

} // namespace module_ns

#endif //_FILTER_INTERFACE_HPP