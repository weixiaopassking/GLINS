/*
 * @Description: 畸变矫正
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */

// eigen
#include <Eigen/Dense>
//自定义速度数据
#include "../../../include/sensor_data/velocity_data.hpp"
//自定义点云数据
#include "../../../include/sensor_data/cloud_data.hpp"

namespace multisensor_localization
{

    class DistortionAdjust
    {
    public:
        void SetMotionParam(float scan_period, VelocityData velocity_data);
        bool AdjustCloud(CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &output_cloud_ptr);

    private:
        Eigen::Matrix3f UpdateMatrix(float time_rotation);

    private:
        float scan_period_;
        Eigen::Vector3f linear_velocity_;
        Eigen::Vector3f angular_velocity_;
    };

} // namespace multisensor_localization
