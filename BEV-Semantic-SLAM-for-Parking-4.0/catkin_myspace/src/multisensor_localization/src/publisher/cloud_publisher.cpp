/*
 * @Description: 点云数据发布
  * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */


// relevent
#include "../../include/publisher/cloud_publisher.hpp"
// pcl
#include <pcl_conversions/pcl_conversions.h>

namespace multisensor_localization
{
    /**
     * @brief 点云发布初始化
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
     * @brief 点云发布消息
     * @note 重载(时间戳取外参)
     * @todo
     **/
    void CloudPublisher::Publish(CloudData::CLOUD_PTR &cloud_ptr_input, double time)
    {
        ros::Time ros_time((float)time);
        PublishData(cloud_ptr_input, ros_time);
    }

    /**
     * @brief 点云发布消息
     * @note 重载(时间戳取发布时刻) 
     * @todo
     **/
    void CloudPublisher::Publish(CloudData::CLOUD_PTR &cloud_ptr_input)
    {
        ros::Time time = ros::Time::now();
        PublishData(cloud_ptr_input, time);
    }

    /**
     * @brief 数据发布
     * @note 
     * @todo
     **/
    void CloudPublisher::PublishData(CloudData::CLOUD_PTR &cloud_ptr_input, ros::Time time)
    {
        /*pcl格式的点云转换为ros类型的消息*/
        sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);
        /*填充数据发布*/
        cloud_ptr_output->header.stamp = time;
        cloud_ptr_output->header.frame_id = frame_id_;
        publisher_.publish(*cloud_ptr_output);
    }

    /**
     * @brief 检查是否被订阅
     * @note
     * @todo
     **/
    bool CloudPublisher::HasSubscribers()
    {
        return publisher_.getNumSubscribers() != 0;
    }


} // namespace lidar_localization