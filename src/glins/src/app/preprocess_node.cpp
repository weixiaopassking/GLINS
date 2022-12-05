/*
 * @Description: preprocess sensor data
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-30
 */

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
// thirdpart lib

// relevent
#include "../../include/preprocess/PreprocessFlow.hpp"
// tools
#include "../../include/tools/ColorTerminal.hpp"

using namespace glins;

void mySigintHandler(int sig)
{
    ColorTerminal::NodeInfo("[preprocess_node] shutdown");
    ros::shutdown();
}

int main(int argc, char **argv)
{

    /*config ros */
    ros::init(argc, argv, "preprocess_node", ros::init_options::NoSigintHandler);
    ros::start();
    ros::NodeHandle nh;
    ros::Rate rate(100);
    signal(SIGINT, mySigintHandler);
    ColorTerminal::NodeInfo("[preprocess_node] start");

    /*config glog*/

    /*config flow*/
    std::shared_ptr<PreprocessFlow> preprocess_flow_ptr = std::make_shared<PreprocessFlow>(nh);

    /*execute circlue */
    while (ros::ok())
    {
        ros::spinOnce();
        preprocess_flow_ptr->Run();
        rate.sleep();
    }

    return 0;
}