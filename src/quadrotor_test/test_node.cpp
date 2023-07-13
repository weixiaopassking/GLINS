#include "./basic_usage/pointcloud_handle/pointcloud_handle.hpp"
#include <Eigen/Core>
#include <memory>
#include <random>
#include <string>

constexpr auto pcd_path = "/home/g/workspace/AlkaidQuadrotor/data/map_example.pcd";
int main(int argc, char **argv)
{
    std::shared_ptr cloud_handle_ptr = std::make_shared<PointCloudHandle>(pcd_path);
    // cloud_handle_ptr->GenerateBevImage();
    // cloud_handle_ptr->Display();
    // std::cout << *cloud_handle_ptr << std::endl;

    /*test for linear fiting*/
    Eigen::Vector3d true_line_stat_point(0.1, 0.3, 0.5);
    Eigen::Vector3d true_line_direction(0.4, 0.5, 0.3);
    const int points_num = 10;

    std::random_device rd;                      // 用于随机数引擎获得随机种子
    std::mt19937 gen((unsigned int)time(NULL)); // 梅森旋转算法（Mersenne twister）
    std::uniform_real_distribution<double> distribute(1, 6);
    for (int i = 0; i < points_num; i++)
    {
        double random = distribute(gen); // 随机数
        std::cout << random << std::endl;
    }
    //  PointCloudHandle::LineFitting();

    return 0;
}