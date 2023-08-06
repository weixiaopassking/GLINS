/*
 * @Description: localization_node
 * @Function: dispatch pipeline
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 */

// pipe
#include "../pipe/odom_pipe.hpp"
// tools
#include "../tools/tools.hpp"
// system
#include <ros/ros.h>
#include <signal.h>

/**
 * @brief    response  interrupt signal
 * @param sig
 * @note in line with forecast
 **/
void MonitorHandler(int sig)
{
    tools_ns::StatusAssert("breakdown by signal");
    ros::shutdown();
}

/**
 * @brief    main entrance
 * @param
 * @note
 **/
int main(int argc, char **argv)
{
    /*1--ros config*/
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle localization_handle;
    signal(SIGINT, MonitorHandler);

    /*2--pipe init*/
    std::shared_ptr<pipe_ns::OdomPipe> odom_pip_ptr = std::make_shared<pipe_ns::OdomPipe>(localization_handle);
    
    /*3--pipe run */
    while (ros::ok())
    {
        bool run_status = odom_pip_ptr->Run();
        if (run_status == false)
        {
            raise(SIGINT);
        }

        ros::spinOnce();
    }
    return 0;
}