/*
 * @Description: odom pipe
 * @Function: dispatch module and others
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 */

#ifndef _ODOM_PIPE_HPP
#define _ODOM_PIPE_HPP
//tools
#include "../tools/tools.hpp"
// data
#include "../data/cloud_data.hpp"
#include "../data/frame_data.hpp"
#include "../data/geometry_data.hpp"
#include "../data/gnss_data.hpp"
// module
#include "../module/cloud_filter/cloud_filter_interface.hpp"
#include "../module/cloud_filter/voxel_filter.hpp"
#include "../module/cloud_registration/cloud_registration_interface.hpp"
#include "../module/cloud_registration/icp_registration.hpp"
#include "../module/cloud_registration/ndt_pcl_registration.hpp"
// sub and pub
#include "../pub/cloud_pub.hpp"
#include "../pub/odom_pub.hpp"
#include "../sub/cloud_sub.hpp"
#include "../sub/gnss_sub.hpp"
// thirdparty lib
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

namespace pipe_ns
{
class OdomPipe
{
  public:
    OdomPipe() = delete; // must use nh
    OdomPipe(ros::NodeHandle &nh);
    bool Run();
    ~OdomPipe()=default;

  private:
    bool AcquireSensorData();
    bool CalculateOdom();
    bool UpdateLocalMap();
    void PublishEstimatedState();

  private:
    // sub and pub
    std::shared_ptr<sub_ns::CloudSub> _cloud_sub_ptr=nullptr;
    std::shared_ptr<sub_ns::GNSSSub> _gnss_sub_ptr=nullptr;
    std::shared_ptr<pub_ns::CloudPub> _cloud_pub_ptr=nullptr;
    std::shared_ptr<pub_ns::OdomPub> _odom_pub_ptr=nullptr;

    // module
    std::shared_ptr<module_ns::CloudRegistrationInterface> _registration_ptr=nullptr;
    std::shared_ptr<module_ns::CloudFilterInterface> _filter_ptr=nullptr;

    // data queue
    std::deque<data_ns::GNSSData> _gnss_data_deq;
    std::deque<data_ns::CloudData> _cloud_data_deq;
    std::deque<data_ns::FrameData> _frame_data_deq;

    // current data
    data_ns::CloudData _cur_cloud_data;
    data_ns::GNSSData _cur_gnss_data;
    data_ns::FrameData _cur_frame_data;

    // flag
    bool _hasGNSSInited = false;

    // local map
    data_ns::CloudData::CLOUD_PTR _local_map_ptr;

}; // class OdomPipe
} // namespace pipe_ns

#endif //_ODOM_PIPE_HPP
