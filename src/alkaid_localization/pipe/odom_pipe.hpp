#ifndef _ODOM_PIPE_HPP
#define _ODOM_PIPE_HPP

//data
#include "../data/cloud_data.hpp"
#include "../data/gnss_data.hpp"
#include "../data/geometry_data.hpp"
//module
#include "../module/cloud_filter/voxel_filter.hpp"
#include "../module/cloud_filter/cloud_filter_interface.hpp"
#include "../module/cloud_registration/ndt_registration.hpp"
#include "../module/cloud_registration/cloud_registration_interface.hpp"
//sub and pub
#include "../pub/cloud_pub.hpp"
#include "../pub/odom_pub.hpp"
#include "../sub/cloud_sub.hpp"
#include "../sub/gnss_sub.hpp"
//thirdparty lib
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

namespace pipe_ns
{
class OdomPipe
{

    struct Frame
    {
        data_ns::Mat4f pose = data_ns::Mat4f::Identity();
        data_ns::CloudData cloud_data;
    };

  public:
    OdomPipe() = delete; // must be init by passing  nodehandle into function so delete defalut
    OdomPipe(ros::NodeHandle &nh);
    bool Run();
    ~OdomPipe();

  private:
    bool UpdateOdom(data_ns::CloudData &cloud_data, data_ns::Mat4f &pose);
    bool AddNewFrame(const Frame &new_key_frame);

  private:
  //sub and pub 
    std::shared_ptr<sub_ns::CloudSub> _cloud_sub_ptr;
    std::shared_ptr<sub_ns::GNSSSub> _gnss_sub_ptr;
    std::shared_ptr<pub_ns::CloudPub> _cloud_pub_ptr;
    std::shared_ptr<pub_ns::OdomPub> _odom_pub_ptr;

    //module
    std::shared_ptr<module_ns::CloudRegistrationInterface> _registration_ptr;
    std::shared_ptr<module_ns::CloudFilterInterface> _filter_ptr;

  private:
    std::deque<data_ns::GNSSData> _gnss_data_deq;
    bool _hasGnssInited=false;



}; // OdomPipe
} // namespace pipe_ns

#endif //_ODOM_PIPE_HPP

// std::deque<Frame> _local_map_deq;
// data_ns::CloudData::CLOUD_PTR _local_map_ptr;
// Frame _current_frame;
// const float _key_frame_distance = 1.0;
// const int _local_frame_num = 20;

// data_ns::Mat4f _pose = data_ns::Mat4f::Identity();
// std::deque<data_ns::CloudData> _cloud_data_deq;