/*
 * @Description: 传感器数据预处理
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-30
 * @Moify: 2023-02-12
 */

#include "../../include/preprocess/PreprocessFlow.hpp"
// ros库
#include <ros/ros.h>
#include <ros/package.h>
// c++库
#include <signal.h>
// 第三方库

// 调试工具
#include "../../include/tools/ColorTerminal.hpp"

using namespace glins;

/**
 * @brief  中断检测
 * @note
 * @todo
 **/
void mySigintHandler(int sig)
{
    ColorTerminal::NodeInfo("[preprocess_node] shutdown");
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{

    /*ros配置 */
    ros::init(argc, argv, "preprocess_node", ros::init_options::NoSigintHandler);
    ros::start();
    ros::NodeHandle nh;
    ros::Rate rate(100);
    signal(SIGINT, mySigintHandler);
    ColorTerminal::NodeInfo("[preprocess_node] start");

    /*日志配置*/

    /*流程控制层创建*/
    std::shared_ptr<PreprocessFlow> preprocess_flow_ptr = std::make_shared<PreprocessFlow>(nh);

    /*执行循环 */
    while (ros::ok())
    {
        ros::spinOnce();
        preprocess_flow_ptr->Run();
        rate.sleep();
    }

    return 0;
}