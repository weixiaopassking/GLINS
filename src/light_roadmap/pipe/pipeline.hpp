#ifndef _PIPE_LINE_HPP
#define _PIPE_LINE_HPP

#include "../sub/gnss_sub.hpp"
#include "../sub/sub_base.hpp"
#include "yaml-cpp/yaml.h"
#include <memory>
#include <ros/ros.h>

namespace pipeline
{

class PipeLine
{
  public:
    PipeLine();
    ~PipeLine();
    PipeLine(const ros::NodeHandle &nh);
    bool Run();

  private:
    std::shared_ptr<sub::SubBase> _gnss_sub_ptr = nullptr;
    YAML::Node _yaml_node;
    std::string _pkg_path;
};

} // namespace pipeline

#endif //_PIPE_LINE_HPP