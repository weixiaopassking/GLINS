/*
 * @Description: OptimizationFlow
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-12-12
 */

#ifndef OPTIMIZATION_FLOW_HPP_
#define OPTIMIZATION_FLOW_HPP_

// ros lib
#include <ros/ros.h>

namespace glins
{
    class OptimizationFlow
    {
    public:
        OptimizationFlow(ros::NodeHandle &nh);
        bool Run();

    private:
        bool CheckDataQueue();
        bool ExtractData();

        void PublishData();

    private:
    };
} // namespace glins

#endif