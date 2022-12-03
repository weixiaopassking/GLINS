
# value define
work_path=/home/tdt/1_niuwengang/GLINS/catkin_glins
add_source="source devel/setup.bash"

gnome-terminal -t "" -x bash -c "cd $work_path;$add_source;echo "hope no bug";exec bash;"
gnome-terminal -t "Imu" -x bash -c "cd $work_path;$add_source;roslaunch fdilink_ahrs ahrs_data.launch;exec bash;"
sleep 2s
gnome-terminal -t "Gnss" -x bash -c "cd $work_path;$add_source;roslaunch ublox_driver ublox_driver.launch;exec bash;"
gnome-terminal -t "Camera" -x bash -c "cd $work_path;$add_source;roslaunch zed_cpu_ros zed_cpu_ros.launch;exec bash;"
gnome-terminal -t "lidar" -x bash -c "cd $work_path;$add_source;roslaunch velodyne_pointcloud VLP16_points.launch;exec bash;"
gnome-terminal -t "bag" -x bash -c "cd /media/tdt/T1001;rosbag record /camera/left/image_raw /euler_angles /magnetic /imu /ublox_driver/ephem /ublox_driver/glo_ephem /ublox_driver/iono_params /ublox_driver/range_meas /ublox_driver/receiver_lla /ublox_driver/receiver_pvt /ublox_driver/time_pulse_info /velodyne_points;exec bash;"
gnome-terminal -t "rviz" -x bash -c "cd $work_path;$add_source;roslaunch glins record.launch;exec bash;"
