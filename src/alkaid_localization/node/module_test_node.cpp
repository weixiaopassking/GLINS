/*
 * @Description: module_test
 * @Function: tests for module
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 */

// data
#include "../data/geometry_data.hpp"
// module
#include "../module/cloud_filter/cloud_filter_interface.hpp"
#include "../module/cloud_filter/voxel_filter.hpp"
#include "../module/cloud_registration/cloud_registration_interface.hpp"
#include "../module/cloud_registration/icp_registration.hpp"
#include "../module/cloud_registration/ndt_pcl_registration.hpp"
#include "../module/imu_preintegration/imu_preintergration.hpp"
// tools
#include "../tools/tools.hpp"
// system
#include <fstream>
#include <iostream>
#include <ros/package.h>
// thirdparty
#include <gtest/gtest.h> //for unit test
#include <pcl/io/pcd_io.h>

TEST(Instance2, imu_preintergration)
{
    std::cout << "\n This is instance2 :test imu preintergration \n" << std::endl;

    const double imu_time_span = 0.01;                 // imu measurment 10ms
    const data_ns::Vec3f angular_velocity(0, 0, M_PI); // 180/s
    const data_ns::Vec3f gravity(0, 0, -9.8);          // z is up dir

    FrameType start_frame(0), end_frame(1);
    // 1.directly intergation
    data_ns::SO3f R;
    data_ns::Vec3f t = data_ns::Vec3f::Zero();
    data_ns::Vec3f v = data_ns::Vec3f::Zero();
    // 2. pre intergation
    module_ns::IMUIntegration imu_integration;

    for (int i = 0; i <= 100; i++)
    {
        double time = imu_time_span * i;
        data_ns::Vec3f accel = -gravity;

        // 1.directly intergation

        // 2. pre intergation
    }
}

/**
 * @brief    main entrance
 * @param
 * @note
 **/
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

#if (0)

/**
 * @brief    test for tools
 * @param
 * @note
 **/
TEST(Instance0, tools)
{
    tools_ns::VariableAssert("cars:", 11, "quadrotors:", 34);
    tools_ns::StatusAssert("status exmaple", __FUNCTION__);
    tools_ns::ErrorAssert("error example", __FILE__, __FUNCTION__, __LINE__);
}

/**
 * @brief    compare with different  cloud_registration's methods
 * @param
 * @note
 **/
TEST(Instance1, cloud_registration)
{
    /*1--get origin pointcloud and ground truth*/
    std::string resource_file_path = ros::package::getPath("alkaid_localization") + "/resource/";
    std::ifstream gt_stream(resource_file_path + "EPFL/aquarius_pose.txt");
    data_ns::Mat4f gt_maxtrix = data_ns::Mat4f::Identity();
    if (gt_stream.is_open())
    {
        double tx, ty, tz, qw, qx, qy, qz;
        gt_stream >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
        gt_stream.close();

        gt_maxtrix.block<3, 3>(0, 0) = data_ns::Quatf(qw, qx, qy, qz).toRotationMatrix();
        gt_maxtrix.block<3, 1>(0, 3) << tx, ty, tz;
        tools_ns::VariableAssert("gt_maxtrix:\n", gt_maxtrix);
    }

    data_ns::CloudData::CLOUD_PTR source_cloud_ptr(new data_ns::CloudData::CLOUD);
    data_ns::CloudData::CLOUD_PTR target_cloud_ptr(new data_ns::CloudData::CLOUD);

    pcl::io::loadPCDFile(resource_file_path + "EPFL/aquarius_source.pcd", *source_cloud_ptr);
    pcl::io::loadPCDFile(resource_file_path + "EPFL/aquarius_target.pcd", *target_cloud_ptr);

    /*2--create method and filter point cloud*/
    std::shared_ptr<module_ns::CloudRegistrationInterface> registration_ptr =
        std::make_shared<module_ns::ICPRegistration>();

    std::shared_ptr<module_ns::CloudFilterInterface> filter_ptr = std::make_shared<module_ns::VoxelFilter>(0.1);
    filter_ptr->Filter(source_cloud_ptr, source_cloud_ptr);
    filter_ptr->Filter(target_cloud_ptr, target_cloud_ptr);

    /*3-compare different methdos on  time*/
    data_ns::Mat4f res_matrix;

    tools_ns::TimeCost(
        [&]() {
            registration_ptr->SetTargetCloud(target_cloud_ptr);
            registration_ptr->SetSourceCloud(source_cloud_ptr);
            res_matrix = registration_ptr->GetResTransform(data_ns::Mat4f::Identity());
        },
        "classic icp", 100);
    tools_ns::VariableAssert("res_matrix:\n", res_matrix);

    registration_ptr = std::make_shared<module_ns::NDTPclRegistration>();
    tools_ns::TimeCost(
        [&]() {
            registration_ptr->SetTargetCloud(target_cloud_ptr);
            registration_ptr->SetSourceCloud(source_cloud_ptr);
            res_matrix = registration_ptr->GetResTransform(data_ns::Mat4f::Identity());
        },
        "classic ndt", 100);
    tools_ns::VariableAssert("res_matrix:\n", res_matrix);
}

#endif