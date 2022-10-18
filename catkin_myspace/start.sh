#启动roscore
#gnome-terminal -t "roscore" -x bash -c "roscore;exec bash;"

work_path=/home/g/workspace/multisensor_fusion_localization_study/catkin_myspace/
add_source="source devel/setup.bash"

#rviz可视化
gnome-terminal -t "rviz" -x bash -c "cd $work_path;$add_source;roslaunch multisensor_localization test_frame.launch;exec bash;"
sleep 2s
#数据预处理节点启动
gnome-terminal -t "node1" -x bash -c "cd $work_path;$add_source;rosrun multisensor_localization data_pretreat_node;exec bash;"
#前端里程计节点启动
sleep 2s
gnome-terminal -t "node2" -x bash -c "cd $work_path;$add_source;rosrun multisensor_localization front_end_node;exec bash;"
#bag启动
sleep 2s
gnome-terminal -t "node3" -x bash -c "cd $work_path;$add_source; rosbag play  kitti_2011_10_03_drive_0027_synced.bag ;exec bash;"
gnome-terminal -t "node4" -x bash -c "cd $work_path;$add_source; rostopic hz /laser_odom ;exec bash;"                           