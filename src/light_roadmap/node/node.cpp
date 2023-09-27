#include "../pipeline/pipeline.hpp"
#include <memory>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::NodeHandle nh;
    std::shared_ptr<PipeLine> pipeline_ptr = std::make_shared<PipeLine>(nh);
    std::cout << "this is demo ";
    return 0;
}