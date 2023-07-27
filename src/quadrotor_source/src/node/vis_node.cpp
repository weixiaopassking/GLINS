// #include <opencv2/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include "bits/stdc++.h"
#include <execution>
#include <ros/ros.h>
#include <time.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;
    // cv::Mat image(100, 200, CV_8UC3, cv::Scalar(255, 255, 255));
    // cv::imshow("./bev.png", image);
    // cvWaitKey(0);
    std::vector<int> vec{1, 2, 3, 4, 5, 6};
     std::for_each(std::execution::par_unseq, vec.begin(), vec.end(), [](int& index) {
        return  (index *=2);});
        for(auto &it:vec)
        {

            sleep(1); // 程序挂起10s
            std::cout<<it<<" "<<std::endl;
        }
}