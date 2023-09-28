#include "pipeline.hpp"

PipeLine::PipeLine(ros::NodeHandle &nh)
{
    gnss_sub_ptr_ = std::make_shared<sub::GnssSub>();
}

bool PipeLine::Run()
{
    return true;
}
