/*
 * @Description: definition for point cloud
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-07
 */


#include "cloud_data.hpp"

namespace data_ns
{

/**
 * @brief    init cloud ptr
 * @param none
 * @note
 **/
CloudData::CloudData()
{
    _cloud_ptr.reset(new CLOUD);
}
} // namespace data_ns
