#include "pipeline.hpp"
#include <ros/package.h>

namespace pipeline
{

PipeLine::PipeLine()
{
}

PipeLine::PipeLine(const ros::NodeHandle &nh)
{
    _gnss_sub_ptr = std::make_shared<sub::GnssSub>();
    _pkg_path = ros::package::getPath("light_roadmap");
    
}

PipeLine::~PipeLine()
{
}

bool PipeLine::Run()
{
    return true;
}

} // namespace pipeline
