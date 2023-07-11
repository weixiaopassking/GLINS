#include "./basic_usage/pointcloud_handle/pointcloud_handle.hpp"
#include <memory>
#include <string>

constexpr auto pcd_path = "/home/g/workspace/AlkaidQuadrotor/data/map_example.pcd";
int main(int argc, char **argv)
{
    std::shared_ptr cloud_handle_ptr = std::make_shared<PointCloudHandle>(pcd_path);
    cloud_handle_ptr->GenerateBevImage(); 
    cloud_handle_ptr->Display();
    std::cout << *cloud_handle_ptr << std::endl;
    return 0;
}