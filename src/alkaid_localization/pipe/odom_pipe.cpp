#include "odom_pipe.hpp"

namespace pipe_ns
{
OdomPipe::OdomPipe(ros::NodeHandle &nh)
{
    std::cout << "[OdomPipe]$ init" << std::endl;

    _cloud_sub_ptr = std::make_shared<sub_ns::CloudSub>(nh, "points_raw", 10);
    _cloud_pub_ptr = std::make_shared<pub_ns::CloudPub>(nh, "points_handled", "map", 1);
}
bool OdomPipe::Run()
{
    _cloud_sub_ptr->ParseData(_cloud_data_que);
    if (_cloud_data_que.size() > 0)
    {
        data_ns::CloudData::CLOUD_PTR  current_cloud_ptr = _cloud_data_que.front()._cloud_ptr;
        _cloud_data_que.pop_front();
      //  _cloud_pub_ptr->Pub(current_cloud_ptr);
    }
    return true;
}

OdomPipe ::~OdomPipe()
{
    std::cout << "[OdomPipe]$ release" << std::endl;
}

bool OdomPipe::UpdateOdom(const data_ns::CloudData &cloud_data, data_ns::Mat4f &pose)
{
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data._cloud_ptr, *cloud_data._cloud_ptr, indices);
    return true;
}

} // namespace pipe_ns
