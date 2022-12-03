#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/gnss_ros_gui/qnode.hpp"

namespace gnss_ros_gui {

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"gnss_ros_gui");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start();
    ros::NodeHandle n;

    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    start();
    return true;
}


void QNode::run() {
    ros::Rate loop_rate(1);
    while ( ros::ok() ) {
        std_msgs::String str_msg;
        std::string str="gui showing ...";
        str_msg.data=str.c_str();
        chatter_publisher.publish(str_msg);
        loop_rate.sleep();
    }


}




}  // namespace gnss_ros_gui
