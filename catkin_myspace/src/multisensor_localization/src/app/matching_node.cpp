/*
 * @Description: 地图匹配
  * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-11-08
 */

// ros库文件
#include <ros/ros.h>
#include <ros/package.h>
// glog日志库
#include <glog/logging.h>
// tools 
#include "../../include/tools/color_terminal.hpp"
//匹配端任务管理器


using namespace multisensor_localization;

int main(int argc, char **argv)
{

    /*ros系统配置*/
    ros::init(argc, argv, "matching_node");
    ros::NodeHandle nh;

    ColorTerminal::ColorNodeInfo("matching_node节点启动");

    /*glog配置*/
    google::InitGoogleLogging(argv[0]);
    std::string path = ros::package::getPath("multisensor_localization");
    FLAGS_log_dir = path + "/Log";
    FLAGS_alsologtostderr = 1;

    //std::shared_ptr<MatchingFlow> matching_flow_ptr=std::make_shared<MatchingFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
      
       rate.sleep();
    }
    return 0;

}
