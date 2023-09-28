#ifndef _PIPE_LINE_HPP
#define _PIPE_LINE_HPP

#include "../sub/gnss_sub.hpp"
#include "../sub/sub_base.hpp"
#include <memory>
#include <ros/ros.h>

class PipeLine
{
  public:
    PipeLine() = delete;
    PipeLine(ros::NodeHandle &nh);
    bool Run();

    ~PipeLine() = default;

    std::shared_ptr<sub::SubBase> gnss_sub_ptr_;
}; // PipeLine

#endif //_PIPE_LINE_HPP