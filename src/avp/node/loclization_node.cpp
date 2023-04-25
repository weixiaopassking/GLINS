#include <ros/ros.h>
#include <memory>
#include "../algorithm/pipeline/service.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "avp_engine");
    ros::NodeHandle nh;


    std::unique_ptr<service> service_ptr = std::make_unique<service>();
    service_ptr->getPipeline("定位");

    return 0;
}