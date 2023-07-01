#include "pointcloud_handle/pointcloud_handle.hpp"
#include <iostream>
#include <memory>


int main()
{
    std::cout << "this is a demo" << std::endl;
    std::string path = "/home/g/workspace/AlkaidQuadrotor";
    std::shared_ptr<quadrotor_test::PointCloudHandle> pointcloud_handle_ptr =
        std::make_shared<quadrotor_test::PointCloudHandle>("gggggg");
}