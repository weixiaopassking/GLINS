/*
 * @Description: odom pipe
 * @Function: dispatch module and others
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 */

#include "odom_pipe.hpp"

namespace pipe_ns
{

/**
 * @brief   odompipe init
 * @param nh ros::NodeHandle
 * @note
 **/
OdomPipe::OdomPipe(ros::NodeHandle &nh)
{

    /*1--ros sub and pub config*/
    _cloud_sub_ptr = std::make_shared<sub_ns::CloudSub>(nh, "points_raw", 1e4);
    _gnss_sub_ptr = std::make_shared<sub_ns::GNSSSub>(nh, "gps_fix", 1e4);
    _cloud_pub_ptr = std::make_shared<pub_ns::CloudPub>(nh, "points_handled", "map", 1e2);
    _odom_pub_ptr = std::make_shared<pub_ns::OdomPub>(nh, "lidar_odom", "map", "ego", 1e2);

    /*2--modules config*/
    // _registration_ptr = std::make_shared<module_ns::NDTRegistration>(); //use polymorphisms
    _registration_ptr = std::make_shared<module_ns::ICPRegistration>(); // use polymorphisms
    _filter_ptr = std::make_shared<module_ns::VoxelFilter>(0.6);        // use polymorphisms

    tools_ns::StatusAssert("config init", __FUNCTION__);
}

/**
 * @brief
 * @param
 * @note
 **/
bool OdomPipe::Run()
{

    /*1--parses sensor datar*/
    while (AcquireSensorData() == true)
    {
        /*2--handle raw data*/
        CalculateOdom();
        /*3--publish */
        PublishEstimatedState();
    }
    return true;
}

/**
 * @brief acquire sensor data
 * @param none
 * @note
 **/
bool OdomPipe::AcquireSensorData()
{
    /*1--parese data from buffer*/
    _gnss_sub_ptr->ParseData(_gnss_data_deq);
    _cloud_sub_ptr->ParseData(_cloud_data_deq);

    bool hasReceivedData = false;
    /*2--get current data*/
    /*2.1--get current cloud data*/
    if (_cloud_data_deq.size() > 0)
    {
        _cur_cloud_data = _cloud_data_deq.front();
        _cloud_data_deq.pop_front();
        _cur_frame_data._cloud_ptr.reset(new data_ns::CloudData::CLOUD(*_cur_cloud_data._cloud_ptr)); //! deep copy
        hasReceivedData = true;
    }

    /*2.2--get current gnss data*/
    if (_gnss_data_deq.size() > 0)
    {
        _cur_gnss_data = _gnss_data_deq.front();
        _gnss_data_deq.pop_front();
        if (_hasGNSSInited == false)
        {
            _cur_gnss_data.Init();
            _hasGNSSInited = true;
         
        }
        hasReceivedData = true;
    }
    /*2.3--get no  data*/

    return hasReceivedData;
}

/**
 * @brief calcaulate odom
 * @param none
 * @note
 **/
bool OdomPipe::CalculateOdom()
{
    /*1--remove invalid cloud points*/
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*_cur_frame_data._cloud_ptr, *_cur_frame_data._cloud_ptr, indices);

    /*2--sample cloud points*/
    data_ns::CloudData::CLOUD_PTR filtered_cloud_ptr(new data_ns::CloudData::CLOUD());
    _filter_ptr->Filter(_cur_frame_data._cloud_ptr, filtered_cloud_ptr);

    /*3--static param for predict*/
    static data_ns::Mat4f step_pose = data_ns::Mat4f::Identity();
    static data_ns::Mat4f last_pose = data_ns::Mat4f::Identity();
    static data_ns::Mat4f predict_pose = data_ns::Mat4f::Identity();
    static data_ns::Mat4f last_key_frame_pose = data_ns::Mat4f::Identity();

    /*4--handle first frame*/
    if (_frame_data_deq.size() == 0)
    {
        _cur_frame_data._pose = data_ns::Mat4f::Identity(); // step a: check data complete
        _frame_data_deq.push_back(_cur_frame_data);         // step b: add into que
        _local_map_ptr.reset(new data_ns::CloudData::CLOUD());
        (*_local_map_ptr) += *(_cur_frame_data._cloud_ptr);            // step c: add into local map
        _registration_ptr->SetTargetCloud(_cur_frame_data._cloud_ptr); // step d:set target cloud

        return true;
    }

    /*5--handle  frames after first*/
    else
    {
        /*5.1--get registration result */
        _registration_ptr->SetSourceCloud(filtered_cloud_ptr);                    // use filtered as source
        _cur_frame_data._pose = _registration_ptr->GetResTransform(predict_pose); // param input is const value
        _cur_frame_data.QuatNorm();
        /*5.2--predcit next pose  */
        step_pose = last_pose.inverse() * _cur_frame_data._pose;
        predict_pose = _cur_frame_data._pose * step_pose;
        last_pose = _cur_frame_data._pose;
        /*5.3--update local map*/
        if ((fabs(last_key_frame_pose(0, 3) - _cur_frame_data._pose(0, 3)) +
             fabs(last_key_frame_pose(1, 3) - _cur_frame_data._pose(1, 3)) +
             fabs(last_key_frame_pose(2, 3) - _cur_frame_data._pose(2, 3))) > 0.5)
        {
            _frame_data_deq.push_back(_cur_frame_data); // step b: add into que
            while (_frame_data_deq.size() > 20)
            {
                _frame_data_deq.pop_front();
            }
            _local_map_ptr.reset(new data_ns::CloudData::CLOUD()); // step c: add into local map
            data_ns::CloudData::CLOUD_PTR transformed_cloud_ptr(new data_ns::CloudData::CLOUD());
            for (int index = 0; index < _frame_data_deq.size(); index++)
            {
                pcl::transformPointCloud(*_frame_data_deq.at(index)._cloud_ptr, *transformed_cloud_ptr,
                                         _frame_data_deq.at(index)._pose);

                (*_local_map_ptr) += (*transformed_cloud_ptr);
            }
            data_ns::CloudData::CLOUD_PTR filtered_local_map_ptr(new data_ns::CloudData::CLOUD());
            _filter_ptr->Filter(_local_map_ptr, filtered_local_map_ptr);
            _registration_ptr->SetTargetCloud(filtered_local_map_ptr);

            last_key_frame_pose = _cur_frame_data._pose;
        }
    }

    return true;
}

/**
 * @brief  publish estimated state
 * @param
 * @note
 **/
void OdomPipe::PublishEstimatedState()
{
    _odom_pub_ptr->Pub(_cur_frame_data._pose);
}

} // namespace pipe_ns
