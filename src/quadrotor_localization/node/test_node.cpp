#include "../../common/project_path.h"//工程全局路径
#include "../lib/cloud_handle_module/cloud_io/cloud_io.hpp"
#include <gtest/gtest.h> //单元测试
#include <memory>//智能指针
#include <pcl/point_cloud.h> //点云
#include <pcl/point_types.h> //点


// #include "../../common/time_utils.hpp"
// #include <Eigen/Core>
// #include <opencv2/opencv.hpp>
// #include <random>
// #include <string>

// static void registerCallBack(std::function<void()>const& f)
// {
//     f();
// }
// TEST(cpp11, test)
// {

//     auto f1 = []() -> void { std::cout << "f1" << std::endl; };
//     auto f2 = []() -> void { std::cout << "f2" << std::endl; };
//     registerCallBack(f1);
//     registerCallBack(f2);
// }
// TEST(pointcloud_handle, knn_bfnn)
// {

//     std::cout << "点云数据路径 " << data_path << endl;
//     std::shared_ptr cloud_handle_ptr =
//         std::make_shared<PointCloudHandle>(data_path + "/first.pcd", data_path + "/second.pcd");
//     RuntimeRecord([&cloud_handle_ptr]() -> void { cloud_handle_ptr->Knn(); }, "暴力匹配",10);

//     SUCCEED();
// }

TEST(cloud_handle_module, cloud_io)
{
    const std::string data_file_path = static_cast<std::string>(PROJECT_PATH) + "/data/";
    std::shared_ptr<CloudIO> cloud_io_ptr = std::make_shared<CloudIO>();
    cloud_io_ptr->LoadCloud(data_file_path + "first.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = cloud_io_ptr->GetCloud();
    std::cout << "获取到的点云尺寸:" << cloud_ptr->size() << std::endl;
    std::cout<< *cloud_io_ptr<<std::endl;
     SUCCEED();
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#if 0
TEST(pointcloud_handle, visual)
{
    constexpr auto pcd_path = "/home/g/workspace/AlkaidQuadrotor/data/map_example.pcd";

        std::shared_ptr cloud_handle_ptr = std::make_shared<PointCloudHandle>(pcd_path);
        /*1--Pcd转换Bev*/
        cloud_handle_ptr->GenerateBevImage();
        /*2--3D可视化*/
        cloud_handle_ptr->Display();
        /*3--打印调试*/
        std::cout << *cloud_handle_ptr << std::endl;
}

TEST(pointcloud_handle, plane_fitting)
{
        cv::RNG rng;
        Eigen::Vector4d real_plane_coffs(0.1, 0.2, 0.3, 0.4), esti_plane_coffs;
        real_plane_coffs.normalize();
        std::vector<Eigen::Vector3d> points;
        for (int i = 0; i < 100; i++)
        {
                Eigen::Vector3d p(rng.uniform(0.0, 0.1), rng.uniform(0.0, 0.1), rng.uniform(0.0, 0.1));
                double n4 = -p.dot(real_plane_coffs.head<3>()) / real_plane_coffs[3]; // 归1化n4
                p = p / (n4 + std::numeric_limits<double>::min());                    // 防止除零
                p += Eigen::Vector3d(rng.gaussian(0.01), rng.gaussian(0.01), rng.gaussian(0.01));

                points.emplace_back(p);
        }
        bool flag = PointCloudHandle::PlaneFitting(points, esti_plane_coffs);
        if (flag == true)
        {
                std::cout << "real_plane_coffs: " << real_plane_coffs.transpose() << std::endl;
                std::cout << "esti_plane_coffs: " << esti_plane_coffs.transpose() << std::endl;
        }
        else
        {
                std::cout << "fail to estimate plane params " << std::endl;
        }
}

TEST(pointcloud_handle, line_fitting)
{
        cv::RNG rng;
        Eigen::Vector3d real_line_stat_point(0.1, 0.3, 0.5), esti_line_stat_point;
        Eigen::Vector3d real_line_direction(0.4, 0.5, 0.6), esti_line_direction;
        real_line_direction.normalize(); // 方向向量归一化

        std::vector<Eigen::Vector3d> points;
        for (int i = 0; i < 100; i++)
        {
                Eigen::Vector3d p = real_line_stat_point + real_line_direction * rng.uniform(-1.0, 1.0);
                p += Eigen::Vector3d(rng.gaussian(0.01), rng.gaussian(0.01), rng.gaussian(0.01));

                points.emplace_back(p);
        }
        bool flag = PointCloudHandle::LineFitting(points, esti_line_stat_point, esti_line_direction);
        if (flag == true)
        {
                std::cout << "real origin: " << real_line_stat_point.transpose() << std::endl;
                std::cout << "estimation origin: " << esti_line_stat_point.transpose() << std::endl;
                std::cout << "real direction: " << real_line_direction.transpose() << std::endl;
                std::cout << "estimation direction: " << esti_line_direction.transpose() << std::endl;
        }
        else
        {
                std::cout << "param estimation failed" << std::endl;
        }
}
#endif