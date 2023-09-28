
local_source="source devel/setup.bash" #local source 
dataset_path="/media/g/Elements/dataset/ford/2017-10-26-V2-Log1.bag" #rosbag路径

gnome-terminal --window
gnome-terminal -t "roscore" -x bash -c "roscore;exec bash;"
sleep 1
gnome-terminal -t "dataset" -x bash -c "${local_source};rosbag play ${dataset_path};exec bash;"
gnome-terminal -t "visualize" -x bash -c "${local_source};roslaunch light_roadmap map.launch;exec bash;"

#gnome-terminal -t "lightroadmap" -x bash -c "rosrun rviz rviz -d ./script/project.rviz;exec bash;exec bash;"
