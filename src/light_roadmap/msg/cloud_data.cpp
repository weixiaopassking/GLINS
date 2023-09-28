/*
 * @Description: definition for point cloud
 * @Author: niu_wengang@163.com
 * @Date: 2023-08-07
 * @Modified: 2023-09-27
 */

#include "cloud_data.hpp"

namespace msg
{
CloudData::CloudData()
{
    cloud_ptr.reset();
}


CloudData::~CloudData()
{

}
} // namespace msg
