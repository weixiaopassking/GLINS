/*
 * @Description: publisher point cloud
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-12-05
 * @Note:
 */

// relevent
#include "../../include/publisher/CloudPublisher.hpp"
// ros lib
#include <sensor_msgs/PointCloud2.h>
// thirdpart lib
#include <pcl_conversions/pcl_conversions.h>

namespace glins
{
    /**
     * @brief  config topic for publishing
     * @note
     * @todo
     **/
    CloudPublisher::CloudPublisher(ros::NodeHandle &nh,
                                   std::string topic_name,
                                   std::string frame_id,
                                   size_t buff_size)
        : nh_(nh), frame_id_(frame_id)
    {
        publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
    }

    /**
     * @brief function overloading1
     * @note internal time stamp
     * @todo
     **/
    void CloudPublisher::Publish(CloudDataType::CLOUD_PTR &cloud_ptr_input)
    {
        ros::Time time = ros::Time::now();
        PublishData(cloud_ptr_input, time);
    }

    /**
     * @brief function overloading2
     * @note external time stamp
     * @todo
     **/
    void CloudPublisher::Publish(CloudDataType::CLOUD_PTR &cloud_ptr_input, double time)
    {
        ros::Time ros_time((float)time);
        PublishData(cloud_ptr_input, ros_time);
    }

    /**
     * @brief publish data
     * @note  internal time stamp
     * @todo
     **/
    void CloudPublisher::PublishData(CloudDataType::CLOUD_PTR &cloud_ptr_input, ros::Time time)
    {

        sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

        cloud_ptr_output->header.stamp = time;
        cloud_ptr_output->header.frame_id = frame_id_;
        publisher_.publish(*cloud_ptr_output);
    }

    /**
     * @brief check
     * @note external time stamp
     * @todo
     **/
    bool CloudPublisher::HasSubscribers()
    {
        return publisher_.getNumSubscribers() != 0;
    }
} // namespace glins
