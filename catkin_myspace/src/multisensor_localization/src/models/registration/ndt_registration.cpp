/*
 * @Description:NDT匹配算法
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-17
 */

//relevent
#include "../../../include/models/registration/ndt_registration.hpp"
//yaml
#include <yaml-cpp/yaml.h>
//glog
#include <glog/logging.h>

namespace multisensor_localization
{

    /**
     * @brief 点云匹配初始化
     * @note 重载(yaml文件提供参数)
     * @todo
     **/
    NdtRegistration::NdtRegistration(const YAML::Node &node)
        : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>())
    {

        float res = node["res"].as<float>();
        float step_size = node["step_size"].as<float>();
        float trans_eps = node["trans_eps"].as<float>();
        int max_iter = node["max_iter"].as<int>();

        SetRegistrationParam(res, step_size, trans_eps, max_iter);
    }

    /**
     * @brief 点云匹配初始化
     * @note 重载(参数文件提供参数)
     * @todo
     **/
    NdtRegistration::NdtRegistration(float res, float step_size, float trans_eps, int max_iter)
        : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>())
    {
        SetRegistrationParam(res, step_size, trans_eps, max_iter);
    }

    /**
     * @brief 设置匹配参数
     * @note
     * @todo
     **/
    bool NdtRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter)
    {
        ndt_ptr_->setResolution(res);
        ndt_ptr_->setStepSize(step_size);
        ndt_ptr_->setTransformationEpsilon(trans_eps);
        ndt_ptr_->setMaximumIterations(max_iter);

        LOG(INFO) << std::endl
                  << "[NdtRegistration_param]" << std::endl
                  << "res \t" << res << std::endl
                  << "step_size \t" << step_size << std::endl
                  << "trans_eps \t" << trans_eps << std::endl
                  << "max_iter \t" << max_iter << std::endl;
        return true;
    }

    /**
     * @brief 设置目标点云
     * @note 向target匹配
     * @todo
     **/
    bool NdtRegistration::SetInputTarget(const CloudData::CLOUD_PTR &input_target)
    {
        ndt_ptr_->setInputTarget(input_target);

        return true;
    }

    /**
     * @brief 点云匹配
     * @note
     * @todo
     **/
    bool NdtRegistration::ScanMatch(const CloudData::CLOUD_PTR &input_cloud_ptr,
                                    const Eigen::Matrix4f &predict_pose,
                                    CloudData::CLOUD_PTR &result_cloud_ptr,
                                    Eigen::Matrix4f &result_pose)
    {
        ndt_ptr_->setInputSource(input_cloud_ptr);
        ndt_ptr_->align(*result_cloud_ptr, predict_pose);
        result_pose = ndt_ptr_->getFinalTransformation();

        return true;
    }

}//namespace multisensor_localization