#启动roscore
gnome-terminal -t "roscore" -x bash -c "roscore;exec bash;"
# 延迟5s确保节点注册成功
sleep 5s
#串口
gnome-terminal -t "roborts_uart" -x bash -c "source devel/setup.bash;rosrun avp perception_node ;exec bash;"
#gui




