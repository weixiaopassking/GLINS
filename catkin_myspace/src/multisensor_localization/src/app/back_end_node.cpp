/*
 * @Description: 传感器数据预处理节点
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-18
 */

//后端部分
#include "../../include/mapping/back_end/back_end_flow.hpp"
// ros库文件
#include <ros/ros.h>
#include <ros/package.h>
// glog日志库
#include <glog/logging.h>
//tools
#include "../../include/tools/color_terminal.hpp"

using namespace multisensor_localization;



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
     std::shared_ptr<BackEndFlow> back_end_flow_ptr = std::make_shared<BackEndFlow>(nh);

    ros::Rate rate(10);//后端10hz即可
    while (ros::ok())
    {
        ros::spinOnce();

         back_end_flow_ptr->Run();

        rate.sleep();
    }
    return 0;
}
