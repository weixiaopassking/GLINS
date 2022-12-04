/*
 * @Description: preprocess sensor data
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-30
 */

#ifndef PREPROCESS_FLOW_HPP_
#define PREPROCESS_FLOW_HPP_

// ros lib
#include <ros/ros.h>
// sub
#include "../subscriber/ImuSubscriber.hpp"
#include "../subscriber/GnssFixSubscriber.hpp"
//pub


namespace glins
{
    class PreprocessFlow
    {
    public:
        PreprocessFlow(ros::NodeHandle &nh);
        bool Run();

    private:
        /*raw sensor data subscrier*/
        std::shared_ptr<ImuSubscriber> imu_sub_ptr_;
        std::shared_ptr<GnssFixSubscriber> gnss_fix_sub_ptr_;
        // std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;

    }; // class PreprocessFlow
} // namespace glins

#endif