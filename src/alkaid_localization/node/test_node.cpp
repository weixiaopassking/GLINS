#include "../pipe/odom_pipe.hpp"
#include <ros/ros.h>
#include <signal.h>

void MyHandler(int sig)
{
    std::cout << "[TestNode]$ test node has been closed" << std::endl;
    ros::shutdown();
}

int main(int argc, char **argv)
{
    /*1--ros config*/
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    signal(SIGINT, MyHandler);
    /*2--pipe init*/
    std::shared_ptr<pipe_ns::OdomPipe> odom_pip_ptr = std::make_shared<pipe_ns::OdomPipe>(nh);
    /*2--pipe run */
    while (ros::ok())
    {

        bool run_status= odom_pip_ptr->Run();
        if (run_status == false)
        {
            raise(SIGINT);
        }
        
        ros::spinOnce();
    }
    return 0;
}