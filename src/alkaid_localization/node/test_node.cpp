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
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    signal(SIGINT, MyHandler);

    std::shared_ptr<pipe_ns::OdomPipe> odom_pip_ptr = std::make_shared<pipe_ns::OdomPipe>(nh);
    while (ros::ok())
    {
        ros::spinOnce();
        bool run_status= odom_pip_ptr->Run();
        if (run_status == false)
        {
            raise(SIGINT);
        }
    }
    return 0;
}