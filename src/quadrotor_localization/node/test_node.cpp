#include <gtest/gtest.h> //单元测试
#include <memory>        //智能指针
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h> //点云
#include <pcl/point_types.h> //点

#include "debug_info.hpp" //工程debug调用
#include "project_path.h" //工程全局路径

#include "cloud_filter_interface.hpp"
#include "cloud_io.hpp"
#include "voxel_filter/voxel_filter.hpp"


#include "cloud_registration_interface.hpp"
#include "icp_registration/icp_registration.hpp"

// TEST(cloud_handle_module, baic_use)
// {
//     const std::string data_file_path = static_cast<std::string>(PROJECT_PATH) + "/data/";
//     std::shared_ptr<CloudIO> cloud_io_ptr = std::make_shared<CloudIO>();
//     cloud_io_ptr->LoadCloud(data_file_path + "map_example.pcd");
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = cloud_io_ptr->GetCloud();
//     std::cout << "获取到的点云尺寸:" << cloud_ptr->size() << std::endl;
//     /*滤波*/
//     std::shared_ptr<CloudFilterInterface> cloud_filter_ptr = std::make_shared<VoxelFilter>(0.5, 0.5, 0.5);
//     cloud_filter_ptr->Filter(cloud_ptr, cloud_ptr);
//     std::cout << "滤波1后的点云尺寸:" << cloud_ptr->size() << std::endl;
//     cloud_filter_ptr = std::make_shared<VoxelFilter>(1, 1, 1);
//     cloud_filter_ptr->Filter(cloud_ptr, cloud_ptr);
//     std::cout << "滤波2后的点云尺寸:" << cloud_ptr->size() << std::endl;
//     // CloudViewer::ViewerByPcl(cloud_ptr);
//     // CloudViewer::ViewerByOpencv(cloud_ptr);

//
//     SUCCEED();
// }

TEST(cloud_handle_module, icp)
{
    /*1--读取gt真值*/
    Sophus::SE3d gt_transform;
    const std::string data_file_path = static_cast<std::string>(PROJECT_PATH) + "/data/";
    std::ifstream gt_file_stream(data_file_path + "EPFL/kneeling_lady_pose.txt");
    if (gt_file_stream.is_open())
    {
        double tx, ty, tz, qw, qx, qy, qz;
        gt_file_stream >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
        // VariableInfo("tx", tx, "ty", ty, "tz", tz, "qw", qw, "qx", qx, "qy", qy, "qz", qz);
        gt_file_stream.close();

        gt_transform = Sophus::SE3d(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d( tx, ty, tz));
    }
    else
    {
        ErrorAssert(ErrorCode::error_file, __FILE__, __FUNCTION__, __LINE__);
    }
    /*2--读取source 和target 点云*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(data_file_path + "EPFL/kneeling_lady_source.pcd", *source_cloud_ptr);
    pcl::io::loadPCDFile(data_file_path + "EPFL/kneeling_lady_target.pcd", *target_cloud_ptr);
    /*3--点云配准*/
    std::shared_ptr<CloudRegistrationInterface> cloud_regstration_ptr = std::make_shared<ICPRegistration>();
    cloud_regstration_ptr->SetSourceCloud(source_cloud_ptr);
    cloud_regstration_ptr->SetTargetCloud(target_cloud_ptr);
    cloud_regstration_ptr->SetGtTransform(gt_transform);

    Sophus::SE3d res_transform;

    bool success_flag=cloud_regstration_ptr->GetResTransform(res_transform);

    if (success_flag==true)
    {
        VariableInfo("res_transform q",
                     res_transform.so3().unit_quaternion().coeffs().transpose());// (imag0, imag1, imag2, real)
        VariableInfo("res_transform t", res_transform.translation().transpose());
    }
      
    SUCCEED();
}


// }

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
