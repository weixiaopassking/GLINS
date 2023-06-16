#include "localization_pipeline.hpp"
#include <gflags/gflags.h>
#include <iostream>
#include <memory>

DEFINE_string(sensor_type, "lidar", "传感器类型");
DEFINE_bool(use_gnss, false, "是否开启gnss");
int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_use_gnss == true)
    {
        std::cout << "gnss启用" << std::endl;
    }
    else
    {
        std::cout << "gnss不启用" << std::endl;
    }
    std::cout << "定位节点启动" << std::endl;
    std::shared_ptr<localization_pipeline> pipe = std::make_unique<localization_pipeline>();
}