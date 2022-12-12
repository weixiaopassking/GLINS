/*
 * @Description: optimize opse
 * @Function: support gtsam g2o and ceres
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-12-12
 */


// relevent
#include "../../include/preprocess/PreprocessFlow.hpp"
// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
// thirdpart lib

// tools
#include "../../include/tools/ColorTerminal.hpp"

using namespace glins;

void mySigintHandler(int sig)
{
    ColorTerminal::NodeInfo("[preprocess_node] shutdown");
    ros::shutdown();
     exit (0);
}

int main(int argc, char **argv)
{

    /*config ros */
    ros::init(argc, argv, "optimization_node", ros::init_options::NoSigintHandler);
    ros::start();
    ros::NodeHandle nh;
    ros::Rate rate(100);
    signal(SIGINT, mySigintHandler);
    ColorTerminal::NodeInfo("[optimization_node] start");

    /*config glog*/

    /*config flow*/
    // std::shared_ptr<PreprocessFlow> preprocess_flow_ptr = std::make_shared<PreprocessFlow>(nh);

    /*execute circlue */
    while (ros::ok())
    {
        ros::spinOnce();
        //preprocess_flow_ptr->Run();
        rate.sleep();
    }

    return 0;
}

