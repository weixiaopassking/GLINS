#include "cloud_sub.hpp"
#include "odom_pub.hpp"
#include "direct_lo.hpp"
#include <iostream>

    namespace pipe_ns
{

class LocPipe
{
  public:
    LocPipe(ros::NodeHandle &nh);
    bool Run();
    ~LocPipe();

  private:
    std::shared_ptr<rossub_ns::CloudSub> _cloud_sub_ptr = nullptr;
    std::shared_ptr<rospub_ns::OdomPub> _odom_pub_ptr = nullptr;
    std::shared_ptr<algorithm_ns::DirectLo> _odom_ptr = nullptr;

    std::deque<common_ns::CloudType> _cloud_que;
    common_ns::CloudType _cloud;
    Sophus::SE3d _etimated_pose = Sophus::SE3d();

}; // class  LocPipe
} // namespace pipe_ns
