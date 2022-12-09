/*
 * @Description: preprocess flow
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-12-05
 * @Note:
 */

// relevent
#include "../../include/preprocess/PreprocessFlow.hpp"
// tools
#include "../../include/tools/ColorTerminal.hpp"

namespace glins
{
    /**
     * @brief  flow init
     * @note config topicand param
     * @todo
     **/
    PreprocessFlow::PreprocessFlow(ros::NodeHandle &nh)
    {
        /*load yaml file*/
        std::string config_file_path = ros::package::getPath("glins") + "/config/params.yaml";
        config_node_ = YAML::LoadFile(config_file_path);

        /*config subscriber*/
        imu_sub_ptr_ = std::make_shared<ImuSubscriber>(nh, config_node_["imu_sub_topic"].as<std::string>(), 10e5);
        gnss_fix_sub_ptr_ = std::make_shared<GnssFixSubscriber>(nh, config_node_["gnss_fix_sub_topic"].as<std::string>(), 10e5);
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, config_node_["cloud_sub_topic"].as<std::string>(), 10e5);

        /*config  publisher*/
        cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/synced_cloud", "/velo_link", 100);
        gnss_fix_pub_ptr_ = std::make_shared<OdomPublisher>(nh, "/synced_gnss", "/map", "velo_link", 100);
        imu_pub_ptr_ = std::make_shared<ImuPublisher>(nh, "/synced_imu", 100, "map");
        imu_odom_pub_ptr_=std::make_shared<OdomPublisher>(nh,"/imu_odom");//imu preintergation result
        // TODO  imu preintegration     imu_odom_ptr_
        enu_origin_pub_ptr_ = std::make_shared<EnuPublisher>(nh, "/ref_point_wgs84", 100, "map");


        /*config setting*/
        gnss_enable_ = config_node_["gnss_enable"];

        /*terminal record*/
        ColorTerminal::FlowInfo("[preprocess_flow]  init");
    }

    /**
     * @brief  flow execute
     * @note
     * @todo
     **/
    bool PreprocessFlow::Run()
    {

        SpaceCalibration();

        TimeSynchronization();

        // InitEnuOrigin();

        while (1)
        {
            // CheckDataQueue
            // ExtractDataCurrent()

            /*imu preintegration*/
            ImuPreintegration();
            // PublishResult
        }

        return true;
    }

    /**
     * @brief  software time synchronization
     * @note
     * @todo
     * hardware time synchronization first if supported
     * usually use gnss time as reference
     **/
    bool PreprocessFlow::TimeSynchronization()
    {

        /*create unsynced queue*/
        static std::deque<ImuDataType> unsynced_imu_data_queue;
        static std::deque<GnssFixDataType> unsynced_gnss_fix_data_queue;
        // static std::deque<CloudDataType> unsynced_cloud_data_queue;

        /*read sensor data from sensors' subscriber class*/
        imu_sub_ptr_->ParseData(unsynced_imu_data_queue);
        gnss_fix_sub_ptr_->ParseData(unsynced_gnss_fix_data_queue);
        imu_sub_ptr_->ParseData(unsynced_imu_data_queue);

        /*interpolation refer to lidar time stamp */
        // ImuDataType::TimeSync()
        // GnssDataType::TimeSync();

        /*time sync flag */
        bool static time_sync_flag = false;
        if (time_sync_flag == false)
        {
            ColorTerminal::FlowInfo("[preprocess_flow]  finish time synchronization");
            time_sync_flag = true;
            return true;
        }

        return false;
    }

    /**
     * @brief  calibrate the relative position between sensors
     * @note
     * @todo
     **/
    bool PreprocessFlow::SpaceCalibration()
    {
        static int space_calibration = false;
        if (space_calibration == false)
        {
            ColorTerminal::FlowInfo("[preprocess_flow]  finish space calibration");
            space_calibration = true;
            return true;
        }

        return false;
    }
    bool PreprocessFlow::InitEnuOrigin()
    {
        return true;
    }

    bool PreprocessFlow::CheckDataQueue()
    {
        return true;
    }

    bool PreprocessFlow::ExtractData()
    {
        return true;
    }
    /**
     * @brief  imu preintergation
     * @note
     * @todo
     **/
    void PreprocessFlow::ImuPreintegration()
    {

    }

    void PreprocessFlow::PublishData()
    {
    }

} // namespace glins
