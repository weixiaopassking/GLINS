/*
 * @Description: 回环检测节点
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-20
 */

// flow
#include "../../include/mapping/loop_closing/loop_closing_flow.hpp"
// ros库文件
#include <ros/ros.h>
#include <ros/package.h>
// glog日志库
#include <glog/logging.h>
// tools
#include "../../include/tools/color_terminal.hpp"

using namespace multisensor_localization;

int main(int argc, char **argv)
{

    /*ros系统配置*/
    ros::init(argc, argv, "loop_closing_node");
    ros::NodeHandle nh;
    ColorTerminal::ColorNodeInfo("loop_closing_node节点启动");

    /*glog配置*/
    google::InitGoogleLogging(argv[0]);
    std::string path = ros::package::getPath("multisensor_localization");
    FLAGS_log_dir = path + "/Log";
    FLAGS_alsologtostderr = 1;     //终端输出日志
    FLAGS_colorlogtostderr = true; //允许输出颜色

    /*回环检测--初始化*/
    std::shared_ptr<LoopClosinigFlow> loop_closing_flow_ptr = std::make_shared<LoopClosinigFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        /*回环检测--运行*/
        loop_closing_flow_ptr->Run();

        rate.sleep();
    }
    return 0;
}