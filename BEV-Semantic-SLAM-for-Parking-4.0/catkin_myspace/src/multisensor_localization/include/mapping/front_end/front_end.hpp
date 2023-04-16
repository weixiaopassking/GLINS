/*
 * @Description: 前端算法
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-17
 */

#ifndef FRONT_END_HPP_
#define FRONT_END_HPP_

// c++
#include <deque>
// eigen
#include <Eigen/Dense>
//自定义点云数据类型
#include "../../sensor_data/cloud_data.hpp"
//点云滤波虚继承接口
#include "../../models/cloud_filter/cloud_filter_interface.hpp"
//点云匹配虚继承接口
#include "../../models/registration/registration_interface.hpp"

namespace multisensor_localization
{
    class FrontEnd
    {
    public:
        struct Frame
        {
            Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();
            CloudData cloud_data_;
        };

    public:
        FrontEnd();

        bool UpdateOdometry(const CloudData &cloud_data, Eigen::Matrix4f &cloud_pose);
        bool SetInitPose(const Eigen::Matrix4f &init_pose);

    private:
        bool ConfigFrame(const YAML::Node &config_node);
        bool ConfigRegistrationMethod(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node);
        bool ConfigFilterMethod(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node);

        bool AddNewFrame(const Frame &new_key_frame);

    private:
        std::string data_path_ = "";

        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        std::shared_ptr<RegistrationInterface> registration_ptr_;

        std::deque<Frame> local_map_frames_;

        CloudData::CLOUD_PTR local_map_ptr_;
        Frame current_frame_;

        Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

        float key_frame_distance_ = 2.0;
        int local_frame_num_ = 20;
    };

}//namespace multisensor_localization

#endif