#include "../sub/cloud_sub.hpp"
#include "../pub/cloud_pub.hpp"

#ifndef _ODOM_PIPE_HPP
#define _ODOM_PIPE_HPP

namespace pipe_ns
{
class OdomPipe
{
  public:
    OdomPipe() = delete;//must be init by passing  nodehandle into function
    OdomPipe(ros::NodeHandle &nh);
    bool Run();
    ~OdomPipe();

  private:
    std::shared_ptr<sub_ns::CloudSub> _cloud_sub_ptr = nullptr;
    std::shared_ptr<pub_ns::CloudPub> _cloud_pub_ptr = nullptr; 
    std::deque<data_ns::CloudData> _cloud_data_que;

}; // OdomPipe
} // namespace pipe_ns

#endif //_ODOM_PIPE_HPP