#include "odom_pipe.hpp"

namespace pipe_ns
{
OdomPipe::OdomPipe(ros::NodeHandle &nh)
{

    /*1--ros sub and pub config*/
    _cloud_sub_ptr = std::make_shared<sub_ns::CloudSub>(nh, "points_raw", 100000);
    _gnss_sub_ptr = std::make_shared<sub_ns::GNSSSub>(nh, "gps_fix", 100000);
    _cloud_pub_ptr = std::make_shared<pub_ns::CloudPub>(nh, "points_handled", "map", 100);
    _odom_pub_ptr = std::make_shared<pub_ns::OdomPub>(nh, "lidar_odom", "map", "drone", 100);
    /*2--modules config*/
    _registration_ptr = std::make_shared<module_ns::NDTRegistration>();
    _filter_ptr = std::make_shared<module_ns::VoxelFilter>();

    std::cout << "[OdomPipe]$ has inited" << std::endl;
}
bool OdomPipe::Run()
{
    /*1--ReadData*/
    _gnss_sub_ptr->ParseData(_gnss_data_deq);

    while (_gnss_data_deq.size() > 0)
    {
        data_ns::GNSSData gnss_data = _gnss_data_deq.front();
        _gnss_data_deq.pop_front();
        if (_hasGnssInited == false)
        {
            gnss_data.Init();
            _hasGnssInited = true;
        }
        else
        {
            data_ns::Mat4d pose=data_ns::Mat4d::Identity();
            gnss_data.Update();
            pose.block<3, 1>(0, 3) << gnss_data._local_east , gnss_data._local_north , gnss_data._local_up;
            std::cout << "[Run]$ pose"<<pose << std::endl;
            _odom_pub_ptr->Pub(pose);
        }
    }

    return true;
}

OdomPipe ::~OdomPipe()
{
    std::cout << "[OdomPipe]$ released" << std::endl;
}
} // namespace pipe_ns








// bool OdomPipe::UpdateOdom(data_ns::CloudData &cloud_data, data_ns::Mat4f &pose)
// {

//     /*1--remove the nan points*/
//     std::vector<int> indices;
//     pcl::removeNaNFromPointCloud(*cloud_data._cloud_ptr, *_current_frame.cloud_data._cloud_ptr, indices);

//     /*2--filter cloud points*/
//     data_ns::CloudData::CLOUD_PTR filtered_cloud_ptr(new data_ns::CloudData::CLOUD());
//     _filter_ptr->Filter(_current_frame.cloud_data._cloud_ptr, filtered_cloud_ptr);

//     /*3--define variables for recurrence */
//     static Eigen::Matrix4f step_pose = data_ns::Mat4f::Identity(); // increase value
//     static Eigen::Matrix4f last_pose = data_ns::Mat4f::Identity(); // init value is Identity
//     static Eigen::Matrix4f predict_pose = data_ns::Mat4f::Identity();
//     static Eigen::Matrix4f last_key_frame_pose = data_ns::Mat4f::Identity();

//     /*4--for first frame*/
//     if (_local_map_deq.size() == 0)
//     {
//         _current_frame.pose = data_ns::Mat4f::Identity();
//         AddNewFrame(_current_frame);
//         pose = _current_frame.pose;
//         return true;
//     }
//     /*5--not first frame*/
//     _registration_ptr->SetSourceCloud(filtered_cloud_ptr);
//     _registration_ptr->GetResTransform(predict_pose);
//     _current_frame.pose = predict_pose; // return the value
//     pose = predict_pose;
//     /*6--update the motion predict*/
//     step_pose = last_pose.inverse() * _current_frame.pose; // increase
//     predict_pose = _current_frame.pose * step_pose;
//     last_pose = _current_frame.pose;

//     /*6--if key frame and update local map*/
//     if ((fabs(last_key_frame_pose(0, 3) - _current_frame.pose(0, 3)) +
//          fabs(last_key_frame_pose(1, 3) - _current_frame.pose(1, 3)) +
//          fabs(last_key_frame_pose(2, 3) - _current_frame.pose(2, 3))) > _key_frame_distance)
//     {
//         AddNewFrame(_current_frame);
//         last_key_frame_pose = _current_frame.pose;
//     }

//     return true;
// }

// bool OdomPipe::AddNewFrame(const Frame &new_key_frame)
// {
//     /*1--deep copy to local*/
//     Frame key_frame = new_key_frame;
//     key_frame.cloud_data._cloud_ptr.reset(new data_ns::CloudData::CLOUD(*new_key_frame.cloud_data._cloud_ptr));
//     data_ns::CloudData::CLOUD_PTR transformed_cloud_ptr(new data_ns::CloudData::CLOUD());

//     /*2--maintain queue size*/
//     _local_map_deq.push_back(key_frame);
//     while (_local_map_deq.size() > static_cast<size_t>(_local_frame_num))
//     {
//         _local_map_deq.pop_front();
//     }
//     _local_map_ptr.reset(new data_ns::CloudData::CLOUD());
//     /*3--stich local map from queue*/
//     for (size_t i = 0; i < _local_map_deq.size(); ++i)
//     {
//         pcl::transformPointCloud(*_local_map_deq.at(i).cloud_data._cloud_ptr, *transformed_cloud_ptr,
//                                  _local_map_deq.at(i).pose);

//         *_local_map_ptr += *transformed_cloud_ptr;
//     }

//     /*4--update the local map*/
//     if (_local_map_deq.size() < 10)
//     {
//         _registration_ptr->SetTargetCloud(_local_map_ptr);
//     }
//     else
//     {
//         data_ns::CloudData::CLOUD_PTR filtered_local_map_ptr(new data_ns::CloudData::CLOUD());
//         _filter_ptr->Filter(_local_map_ptr, filtered_local_map_ptr);
//         _registration_ptr->SetTargetCloud(_local_map_ptr);
//     }
//     return true;
// }
