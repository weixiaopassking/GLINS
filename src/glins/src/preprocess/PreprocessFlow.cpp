/*
 * @Description: preprocess flow
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-12-05
 * @Note:
 */

#include "../../include/preprocess/PreprocessFlow.hpp"
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
        std::string config_file_path = ros::package::getPath("glins") + "/config/preprocess.yaml";
        config_node_ = YAML::LoadFile(config_file_path);
        /*config subscriber*/
        imu_sub_ptr_ = std::make_shared<ImuSubscriber>(nh, config_node_["imu_sub_topic"].as<std::string>(), 10e5);
        gnss_fix_sub_ptr_ = std::make_shared<GnssFixSubscriber>(nh, config_node_["gnss_fix_sub_topic"].as<std::string>(), 10e5);
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, config_node_["cloud_sub_topic"].as<std::string>(), 10e5);
        /*config  publisher*/
        cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/synced_cloud", "/velo_link", 100);
        gnss_fix_pub_ptr_ = std::make_shared<OdomPublisher>(nh, "/synced_gnss", "/map", "velo_link", 100);
        enu_origin_pub_ptr_ = std::make_shared<EnuPublisher>(nh, "/ref_point_wgs84", 100, "map");
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

        // SynceDataQueue();

        // InitCalibration();

        // InitEnuOrigin();

        while (1)
        {
            // CheckDataQueue
            // ExtractDataCurrent()
            // PublishResult
        }

        return true;
    }

    /**
     * @brief  software time synchronization
     * @note
     *  hardware time synchronization first if supported 
     * usually use gnss time as reference
     * @todo 
     **/
    bool PreprocessFlow::TimeSynchronization()
    {
        static std::deque<ImuDataType> unsynced_imu_data_queue;
        static std::deque<CloudDataType> unsynced_cloud_data_queue;
    }

} // namespace glins
