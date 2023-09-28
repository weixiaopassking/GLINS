#include "../pipe/pipeline.hpp"
#include <memory>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::NodeHandle nh;
    std::shared_ptr<pipeline::PipeLine> pipeline_ptr = std::make_shared<pipeline::PipeLine>(nh);
    std::cout << "this is demo ";
    return 0;
}