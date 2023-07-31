#include "loc_pipe.hpp"

namespace pipe_ns
{

LocPipe::LocPipe(ros::NodeHandle &nh)
{
    std::cout << "loc pipe 构造" << std::endl;
    /*1--算法调用初始化*/
    _cloud_sub_ptr = std::make_shared<rossub_ns::CloudSub>(nh, "/points_raw");
    _odom_ptr = std::make_shared<algorithm_ns::DirectLo>();
    _odom_pub_ptr = std::make_shared<rospub_ns::OdomPub>(nh, "odom", "map", "lidar", 1);

}

bool LocPipe::Run()
{
    _cloud_sub_ptr->ParseData(_cloud_que);

    if (_cloud_que.size() != 0)
    {
        _cloud = _cloud_que.front();
        _cloud_que.pop_front();
       _odom_ptr->Update(_cloud._cloud_ptr, _etimated_pose);
       _odom_pub_ptr->Publish(_etimated_pose.matrix().cast<float>());
        std::cout << "当前位姿:" << _etimated_pose.matrix().cast<float>() << std::endl;
    }
    return true;
}

LocPipe::~LocPipe()
{
}

}; // namespace pipe_ns
