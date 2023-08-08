
local_source="source devel/setup.bash"
dataset_path="/home/g/dataset/20130110.bag"

gnome-terminal --window
gnome-terminal -t "roscore" -x bash -c "roscore;exec bash;"
#windows1
gnome-terminal -t "localization" -x bash -c "${local_source};rosrun alkaid_localization localization_node;exec bash;"
gnome-terminal -t "dataset" -x bash -c "rosbag play ${dataset_path};exec bash;"
gnome-terminal -t "rviz" -x bash -c "rosrun rviz rviz -d ./script/project.rviz;exec bash;;exec bash;"

