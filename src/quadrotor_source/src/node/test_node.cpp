#include "loc_pipe.hpp"
#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <signal.h>

void MyHandler(int sig)
{
    std::cout << " 定位节点closed" << std::endl;
    // todo 告知其他节点
    ros::shutdown();
}

int main(int argc, char **argv)
{
    std::cout << " 定位节点loaded" << std::endl;

    ros::init(argc, argv, "test_node_loc");
    ros::NodeHandle nh;
    signal(SIGINT, MyHandler);

    std::shared_ptr<pipe_ns::LocPipe> loc_pipe = std::make_shared<pipe_ns::LocPipe>(nh);

    ros::Rate rate(10); // 10hz执行一次
    while (ros::ok())
    {

        ros::spinOnce();
        // rate.sleep();
        bool run_status = loc_pipe->Run();

        // if (run_status == false)
        // {
        //     raise(SIGINT);
        // }
    }
    return 0;
}