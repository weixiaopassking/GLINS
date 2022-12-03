/*
 * @Description: preprocess sensor data
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-30
 */

#ifndef PREPROCESS_FLOW_HPP_
#define PREPROCESS_FLOW_HPP_

#include <ros/ros.h>

namespace glins
{
    class PreprocessFlow
    {
    public:
        PreprocessFlow(ros::NodeHandle &nh);
        bool Run();
    };//class PreprocessFlow
}//namespace glins

#endif