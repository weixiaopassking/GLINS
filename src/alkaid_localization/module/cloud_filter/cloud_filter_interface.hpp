/*
 * @Description: cloud filter interface
 * @Function: as interface to use different menthods
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 * @Note: supported ndt_pcl、p-p-icp
 */

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
    virtual ~CloudFilterInterface() = default;//must be set virtual to release sub class's memory
};//class CloudFilterInterface

} // namespace module_ns

#endif //_FILTER_INTERFACE_HPP


