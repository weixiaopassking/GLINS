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

//relevent
#include "../../include/preprocess/PreprocessFlow.hpp"

 using namespace glins;

void mySigintHandler(int sig)
{
    std::cout << std::endl
              << "preprocess_node has been shutdown" << std::endl;
    ros::shutdown();
}

int main(int argc, char **argv)
{

    /*ros config */
    ros::init(argc, argv, "preprocess_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::Rate rate(100);
    signal(SIGINT, mySigintHandler);

    /*glog config*/

    /*flow config*/
    //shared_ptr
    while (ros::ok())
    {
        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}