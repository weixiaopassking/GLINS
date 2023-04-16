/*
 * @Description: 可视化节点
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-31
 */

//可视化流程控制
#include "../../include/mapping/viewer/viewer_flow.hpp"
// ros库文件
#include <ros/ros.h>
#include <ros/package.h>
// glog日志库
#include <glog/logging.h>
// tools
#include "../../include/tools/color_terminal.hpp"
//自定义头文件
#include <multisensor_localization/saveMap.h>

using namespace multisensor_localization;

bool save_map_flag = false;

bool SaveMapCallback(saveMap::Request &request, saveMap::Response &response)
{
    std::cout<<"hit "<<std::endl;
  save_map_flag = true;
  response.succeed = true;
  return response.succeed;
}

int main(int argc, char **argv)
{

  /*ros系统配置*/
  ros::init(argc, argv, "viewer_node_node");
  ros::NodeHandle nh;

  ColorTerminal::ColorNodeInfo("viewer_node节点启动");

  /*glog配置*/
  google::InitGoogleLogging(argv[0]);
  std::string path = ros::package::getPath("multisensor_localization");
  FLAGS_log_dir = path + "/Log";
  FLAGS_alsologtostderr = 1;

  /*数据预处理流程指针*/
  std::shared_ptr<ViewerFlow> viewer_flow_ptr = std::make_shared<ViewerFlow>(nh);
  /*消息配置*/
  ros::ServiceServer service = nh.advertiseService("save_map", SaveMapCallback);
  ros::Rate rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
      viewer_flow_ptr->Run();
       if (save_map_flag)
    {
      viewer_flow_ptr->SaveMap();
            save_map_flag = false;
    }
  

    rate.sleep();
  }
  return 0;
}
