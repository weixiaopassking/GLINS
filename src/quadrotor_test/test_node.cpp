#include "./basic_usage/pointcloud_handle/pointcloud_handle.hpp"
#include <Eigen/Core>
#include <memory>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>

constexpr auto pcd_path = "/home/g/workspace/AlkaidQuadrotor/data/map_example.pcd";
int main(int argc, char **argv)
{

    /*1--直线拟合测试*/
    Eigen::Vector3d real_line_stat_point(0.1, 0.3, 0.5);
    Eigen::Vector3d real_line_direction(0.4, 0.5, 0.6);
    real_line_direction.normalize(); // 方向向量归一化
    const int points_num = 100;
    cv::RNG rng;

    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i < points_num; i++)
    {
        Eigen::Vector3d p = real_line_stat_point + real_line_direction * rng.uniform(-1.0, 1.0);
        p += Eigen::Vector3d(rng.gaussian(0.01), rng.gaussian(0.01), rng.gaussian(0.01));

        points.emplace_back(p);
    }
    Eigen::Vector3d esti_line_stat_point, esti_direction;
    bool flag = PointCloudHandle::LineFitting(points, esti_line_stat_point, esti_direction);
    if (flag == true)
    {
        std::cout << "true origin: " << real_line_stat_point.transpose() << std::endl;
        std::cout << "estimation origin: " << esti_line_stat_point.transpose() << std::endl;
        std::cout << "true direction: " << real_line_direction.transpose() << std::endl;
        std::cout << "estimation direction: " << esti_direction.transpose() << std::endl;
    }
    else
    {
        std::cout << "param estimation failed" << std::endl;
    }

    std::shared_ptr cloud_handle_ptr = std::make_shared<PointCloudHandle>(pcd_path);

    /*2--平面拟合*/
    Eigen::Vector4d real_true_coffs(0.1, 0.2, 0.3, 0.4);//不理解
    real_true_coffs.normalized();

    for (int i = 0; i < 100; i++)
    {
        Eigen::Vector3d p(rng.uniform(0.0, 0.1), rng.uniform(0.0, 0.1), rng.uniform(0.0, 0.1));
        double n4 = -p.dot(real_true_coffs.head<3>()) / real_true_coffs[3];//归1化n4
        p = p / (n4 + std::numeric_limits<double>::min()); // 防止除零
        p += Eigen::Vector3d(rng.gaussian(0.01), rng.gaussian(0.01), rng.gaussian(0.01));
        
        points.emplace_back(p);
    }
    /*3--Pcd转换Bev*/
    cloud_handle_ptr->GenerateBevImage();
    /*4--3D可视化*/
    cloud_handle_ptr->Display();

    // std::cout << *cloud_handle_ptr << std::endl;

    return 0;
}
