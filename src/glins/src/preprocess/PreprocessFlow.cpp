/*
 * @Description: preprocess flow
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-30
 */

#include "../../include/preprocess/PreprocessFlow.hpp"

namespace glins
{
    /**
     * @brief  flow init
     * @note config topicand param
     * @todo
     **/
    PreprocessFlow::PreprocessFlow(ros::NodeHandle &nh)
    {

        imu_sub_ptr_ = std::make_shared<ImuSubscriber>(nh, "/imu", 10e6);
    }

    /**
     * @brief  flow execute
     * @note
     * @todo
     **/
    bool PreprocessFlow::Run()
    {
        return true;
    }
}