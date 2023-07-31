#include <iostream>
#include <memory>
#include <ros/ros.h>
#include "loc_pipe.hpp"

int main(int argc, char **argv)
{
    std::cout << " 定位节点loading..." << std::endl;

    ros::init(argc, argv, "test_node_loc");
    ros::NodeHandle nh;

    std::shared_ptr<pipe_ns::LocPipe> loc_pipe = std::make_shared<pipe_ns::LocPipe>();

    ros::Rate rate(10); // 10hz执行一次
    while (ros::ok())
    {
        loc_pipe->Run();
         ros::spinOnce();
        rate.sleep();
    }
}