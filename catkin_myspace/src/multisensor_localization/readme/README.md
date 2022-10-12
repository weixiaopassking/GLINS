# 多传感器融合定位框架(Tag3.0 stable)

### 功能
定位+建图





### 代码编写注意事项
1. 头文件尽可能精准，多余的头文件会导致编译效率下降
2. 文件注释、函数注释注明
3. 及时上传仓库
4. 预处理器搜索的顺序（当前文件所在目录) --> 编译选项-I指定的目录 --> 默认的标准库目录
所以，用户自定义头文件"",系统头文件<>
5. yaml读参录入LOG确认
6. 编译警告尽可能避免 影响编译速度

### 必要说明
1. kitti时间戳 ms.ns 
2. 话题frame查看 rostopic echo /synced_cloud |grep frame_id
3. catkin_make -DCATKIN_WHITELIST_PACKAGES="multisensor_localization" -j8
4. tools存放小工具诸如文件管理器、计时器、彩色终端等


### 开发记录
+ 2022.10.3  
第三次重构框架搭建  
sensor_data完成部分
+ 2022.10.4    
节点data_pretreat_node完成(未测试)  
节点front_end_node完成(未测试)