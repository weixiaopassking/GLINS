/*
 * @Description: 后端节点
 * @Author: Robotic Gang
 * @Note: modified from Ren Qian
 * @Status: 基本可以
 * @Date: 2022-10-11
 */

// ros库文件
#include <ros/ros.h>
#include <ros/package.h>
// glog日志库
#include <glog/logging.h>
//后端部分
#include "../../include/mapping/back_end/back_end_flow.hpp"

using namespace multisensor_localization;

std::shared_ptr<BackEndFlow> back_end_flow_ptr;

int main(int argc, char **argv)
{
    /*ros系统配置*/
    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;

    /*后端优化流程*/
    std::shared_ptr<BackEndFlow> back_end_flow_ptr=std::make_shared<BackEndFlow>(nh);

    /*glog配置*/
    google::InitGoogleLogging(argv[0]);
    std::string path = ros::package::getPath("multisensor_localization");
    FLAGS_log_dir = path + "/Log";
    FLAGS_alsologtostderr = 1;
    /*后端任务管理器创建*/
   back_end_flow_ptr=std::make_shared<BackEndFlow>(nh);

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        //back_end_flow_ptr->Run();

        rate.sleep();
    }
    return 0;
}
