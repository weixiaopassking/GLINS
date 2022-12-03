/*
 * @Description: just a test for imu and gnss data
 * @Function: analyze raw gnss data
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-22
 */

// ros
#include <ros/ros.h>
// sensor data type
#include "../../include/sensor_data/GnssDataType.hpp"
#include "../../include/sensor_data/ImuDataType.hpp"
// c++
#include <deque>
// thirdpart lib
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
// subscriber
#include "../../include/subscriber/GnssFixSubscriber.hpp"
#include "../../include/subscriber/ImuSubscriber.hpp"
// publish
#include "../../include/publisher/EnuPublisher.hpp"
#include "../../include/publisher/OdomPublisher.hpp"

using namespace glins;

std::deque<GnssFixData> gnss_fix_data_deque;
std::deque<ImuData> imu_data_deque;
GnssFixData gnss_fix_data_current;
ImuData imu_data_current;

int main(int argc, char **argv)
{

    ros::NodeHandle nh;

    std::shared_ptr<GnssFixSubscriber> gnss_fix_sub_ptr = std::make_shared<GnssFixSubscriber>(nh, "/ublox_driver/receiver_lla", 1e6);
    std::shared_ptr<ImuSubscriber> imu_sub_ptr = std::make_shared<ImuSubscriber>(nh, "/imu", 1e6);

    std::shared_ptr<EnuPublisher> enu_origin_pub_ptr = std::make_shared<EnuPublisher>(nh, "/ref_point_wgs84", 100, "map");
    std::shared_ptr<OdomPublisher> odom_pub_ptr = std::make_shared<OdomPublisher>(nh, "/gnss_odom", "/map", "/velo_link", 100);
    ros::Rate sleep(10);
    while (ros::ok())
    {
        ros::spinOnce();

        gnss_fix_sub_ptr->ParseData(gnss_fix_data_deque);
        imu_sub_ptr->ParseData(imu_data_deque);

        if (gnss_fix_data_deque.size() > 0)
        {
            gnss_fix_data_current = gnss_fix_data_deque.front();
            gnss_fix_data_deque.pop_front();

            static int flag_fixed = false;
            if (flag_fixed == false)
            {
                gnss_fix_data_current.FixOrigin();
                enu_origin_pub_ptr->Publish(gnss_fix_data_current);
                flag_fixed = true;
                std::cout << "gnss has been fixed " << std::endl;
            }
        }

        if (imu_data_deque.size() > 0)
        {
            imu_data_current = imu_data_deque.front();
            imu_data_deque.pop_front();

            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            gnss_fix_data_current.UpdateGnssOdom();
            T(0, 3) = gnss_fix_data_current.local_east;
            T(1, 3) = gnss_fix_data_current.local_north;
            T.block<3, 3>(0, 0) = imu_data_current.OrientationToRotationMatrix();

            static int flag_imu = false;
            static Eigen::Matrix3f imu_to_world = Eigen::Matrix3f::Identity();
            if (flag_imu == false)
            {
                imu_to_world = T.block<3, 3>(0, 0).inverse();
                flag_imu = true;
            }
            T.block<3, 3>(0, 0) = imu_to_world * T.block<3, 3>(0, 0);
            std::cout << "===================" << std::endl;
            std::cout << T.block<3, 3>(0, 0) << std::endl;
            odom_pub_ptr->Publish(T);
        }
    }
    return 0;
}
