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
    ColorTerminal::FlowInfo("[preprocess_node] shutdown");
    ros::shutdown();
}

int main(int argc, char **argv)
{

    /*ros config */
    ros::init(argc, argv, "preprocess_node", ros::init_options::NoSigintHandler);
    ros::start();
    ros::NodeHandle nh;
    ros::Rate rate(100);
    signal(SIGINT, mySigintHandler);
    ColorTerminal::FlowInfo("[preprocess_node] start");

    /*glog config*/

    /*flow config*/

    while (ros::ok())
    {
        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}