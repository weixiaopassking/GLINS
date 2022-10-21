/*
 * @Description: 回环检测节点
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-20
 */

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

    /*数据预处理任务管理--初始化*/
    //std::shared_ptr<LoopClosingFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        /*数据预处理任务管理--运行*/
       // data_pretreat_flow_ptr->Run();

        rate.sleep();
    }
    return 0;
}