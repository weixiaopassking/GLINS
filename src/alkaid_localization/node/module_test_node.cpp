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

/**
 * @brief    instance3
 * @param
 * @note
 **/
TEST(Instance4, imu_preintergration)
{
    std::cout << "\n This is instance3 :test imu preintergration \n" << std::endl;

    /*1--params preseted*/
    const double delta_t = 0.01; // 10ms sample
    const data_ns::Vec3f gyro(0, 0, M_PI);
    const data_ns::Vec3f gravity(0, 0, -9.8);
    const data_ns::Vec3f accel(0.1, 0.0, -9.8);
    const double start_time = 0.0, end_time = 1.0;

    /*2--variables preseted*/
    /*2.1--directly intergation's variable*/
    data_ns::SO3f R;
    data_ns::Vec3f t = data_ns::Vec3f::Zero();
    data_ns::Vec3f v = data_ns::Vec3f::Zero();
    /*2.2-preintergation's variable*/
    std::shared_ptr<module_ns::IMUPreIntegration> imu_preintegration_ptr =
        std::make_shared<module_ns::IMUPreIntegration>();

    data_ns::FrameData start_state(0.0);
    data_ns::FrameData end_state(1.0);

    /*3--run it*/
    for (int index = 0; index <= 100; index++)
    {
        double time = delta_t * index;

        /*3.1--directly intergation*/
        t = t + v * delta_t + 0.5 * gravity * pow(delta_t, 2) + 0.5 * (R.matrix() * accel) * pow(delta_t, 2);
        v = v + (R.matrix() * accel) * delta_t + gravity * delta_t;
        R = R * data_ns::SO3f::exp(gyro * delta_t);
        // 2. pre intergation
        data_ns::IMUData imu_data;
        imu_data._gyro = gyro;
        imu_data._accel = accel;
        imu_preintegration_ptr->UpdateIMUData(imu_data, delta_t);
    }
    /*4--print result*/
    end_state = imu_preintegration_ptr->UpdateState(start_state, gravity);
    tools_ns::VariableAssert("R", end_state.GetRotation(), "t", end_state.GetTranslation().transpose(), "velocity",
                             end_state._linear_velocity.transpose());
    tools_ns::VariableAssert("R", R.matrix(), "t", t.transpose(), "velocity", v.transpose());
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
TEST(Instance1, tools)
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
TEST(Instance2, cloud_registration)
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


TEST(Instance3, usage_for_sopuhs)
{



}

#endif