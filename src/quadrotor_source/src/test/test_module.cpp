#include "debug_utils.hpp"
#include "project_path.h"
#include <gtest/gtest.h> //单元测试
#include <memory>        //智能指针
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h> //点云
#include <pcl/point_types.h> //点
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

#include "cloud_handle/cloud_registration/cloud_registration_interface.hpp"
#include "cloud_handle/cloud_registration/icp_registration/icp_registration.hpp"

TEST(cloud_handle_module, icp)
{
    /*1--读取gt真值*/
    Sophus::SE3d gt_transform;
    const std::string data_file_path = static_cast<std::string>(PROJECT_PATH) + "/data/";
    std::ifstream gt_file_stream(data_file_path + "EPFL/kneeling_lady_pose.txt");
    common_ns::VariableInfo(data_file_path);
    if (gt_file_stream.is_open())
    {
        double tx, ty, tz, qw, qx, qy, qz;
        gt_file_stream >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
        common_ns::VariableInfo("tx", tx, "ty", ty, "tz", tz, "qw", qw, "qx", qx, "qy", qy, "qz", qz);
        gt_file_stream.close();

        gt_transform = Sophus::SE3d(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
    }
    else
    {
        common_ns::ErrorAssert("真实gt",__FILE__, __FUNCTION__, __LINE__);
    }
    /*2--读取source 和target 点云*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(data_file_path + "EPFL/kneeling_lady_source.pcd", *source_cloud_ptr);
    pcl::io::loadPCDFile(data_file_path + "EPFL/kneeling_lady_target.pcd", *target_cloud_ptr);
    // /*3--点云配准*/
    std::shared_ptr<algorithm_ns::CloudRegistrationInterface> cloud_regstration_ptr = std::make_shared<algorithm_ns::ICPRegistration>();
    cloud_regstration_ptr->SetSourceCloud(source_cloud_ptr);
    cloud_regstration_ptr->SetTargetCloud(target_cloud_ptr);
    cloud_regstration_ptr->SetGtTransform(gt_transform);

    Sophus::SE3d res_transform;

    bool success_flag = cloud_regstration_ptr->GetResTransform(res_transform);

    if (success_flag == true)
    {
        common_ns::VariableInfo(
            "res_transform q",
            res_transform.so3().unit_quaternion().coeffs().transpose()); // (imag0, imag1, imag2, real)
        common_ns::VariableInfo("res_transform t", res_transform.translation().transpose());
    }

    SUCCEED();
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}