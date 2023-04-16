/*
 * @Description: 前端里程计
  * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-17
 */

// ros库文件
#include <ros/ros.h>
#include <ros/package.h>
// glog日志库
#include <glog/logging.h>
// tools 
#include "../../include/tools/color_terminal.hpp"
//数据处理流程控制
#include "../../include/mapping/front_end/front_end_flow.hpp"

using namespace multisensor_localization;

int main(int argc, char **argv)
{

    /*ros系统配置*/
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    ColorTerminal::ColorNodeInfo("front_end_node节点启动");

    /*glog配置*/
    google::InitGoogleLogging(argv[0]);
    std::string path = ros::package::getPath("multisensor_localization");
    FLAGS_log_dir = path + "/Log";
    FLAGS_alsologtostderr = 1;

    /*数据预处理流程指针*/
    std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
       front_end_flow_ptr->Run();
       rate.sleep();
    }
    return 0;

}
