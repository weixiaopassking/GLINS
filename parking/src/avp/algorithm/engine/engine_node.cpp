#include <ros/ros.h>
#include <memory>
#include "../pipeline/service.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "avp_engine");
    ros::NodeHandle nh;

    std::cout << "this is stat" << std::endl;

    std::unique_ptr<service> service_ptr = std::make_unique<service>();

    return 0;
}   