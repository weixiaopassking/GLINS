/*
 * @Description: 传感器数据预处理节点
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-18
 *  @Todo: 最后一次强制优化
 */

//后端部分
#include "../../include/mapping/back_end/back_end_flow.hpp"
// ros库文件
#include <ros/ros.h>
#include <ros/package.h>
// glog日志库
#include <glog/logging.h>
// tools
#include "../../include/tools/color_terminal.hpp"
//自定义头文件
#include <multisensor_localization/optimizeMap.h>

using namespace multisensor_localization;

bool is_need_optimize = false;

/**
 * @brief 优化后地图的回调函数
 * @note
 * @todo
 **/
bool OptimizeMapCallback(optimizeMap::Request &request, optimizeMap::Response &response)
{
    is_need_optimize = true;
    response.succeed = true;
    return response.succeed;
}
std::shared_ptr<BackEndFlow> back_end_flow_ptr;
int main(int argc, char **argv)
{
    /*ros系统配置*/
    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;
    ColorTerminal::ColorNodeInfo("back_end_node节点启动");

    /*glog配置*/
    google::InitGoogleLogging(argv[0]);
    std::string path = ros::package::getPath("multisensor_localization");
    FLAGS_log_dir = path + "/Log";
    FLAGS_logtostderr = true;
    FLAGS_alsologtostderr = 1;
    /*后端任务管理器创建*/
    back_end_flow_ptr = std::make_shared<BackEndFlow>(nh);
    /*创建服务*/
    ros::ServiceServer force_optimize = nh.advertiseService("force_optimize", OptimizeMapCallback);

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        back_end_flow_ptr->Run();

        /*执行强制优化*/
        if (is_need_optimize == true)
        {
            LOG(INFO) << "[强制优化启动]" << std::endl;
            back_end_flow_ptr->ForceOptimize();
            is_need_optimize = false;
        }

        rate.sleep();
    }
    return 0;
}
