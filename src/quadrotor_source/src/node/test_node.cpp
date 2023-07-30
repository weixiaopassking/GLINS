#include <iostream>

#include "cloud_sub.hpp"
#include <memory>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    /*project start*/
    std::shared_ptr<rossub_ns::CloudSub> cloud_sub_ptr = std::make_shared<rossub_ns::CloudSub>(nh, "/points_raw");

    std::cout << "quadrotor 启动入口" << std::endl;

    ros::Rate rate(1); // 1s
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        std::cout << "reading  raw data" << std::endl;
    }
}