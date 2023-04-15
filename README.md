# BEV-Semantic-SLAM-for-Parking

**预期功能**  bev视角下融合lidar、svc等传感器实现简单的代客泊车功能     
**项目说明** 个人学习练手的demo，侧重于工程化实现(代码性能、框架调度等考量)及算法学习(多传感器的使用、语义slam、BEV former、混合astar等)     
**更新说明** tag形式记录
  
**测试视频:**  
 [bilibili](https://space.bilibili.com/356146260/channel/collectiondetail?sid=753064&ctype=0)  

**相关论文:**  
 [GVINS: Tightly Coupled GNSS-Visual-Inertial Fusion for Smooth and Consistent State Estimation](https://arxiv.org/abs/2103.07899)     
[IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System for Wheeled Robot](https://arxiv.org/abs/2204.04962)      
 [LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping](https://arxiv.org/abs/2007.00258)  

**联系方式:**   
如何有问题请提交issue或发邮件至niu_wengang@163.com.

**框架:**

**测试图片:**    
<left class="half">
    <img src="./pic/gnss_test1.jpg" height="100">
    <img src="./pic/gnss_test2.png" height="100"/>
</left>
     
## 1.软件依赖项
**ros melodic**
```shell
#just follow ros wiki for melodic version
```
**cmake  (3.16 or higher is required)**  
```
#不需要移除旧版本的cmake，直接按照以下指令覆盖即可
git clone -b v3.16.5  https://github.com/Kitware/CMake.git
mkdir build && cd build
cmake ..
make 
sudo make install 
```
**GeographicLib**
```shell
git clone https://github.com/geographiclib/geographiclib.git  
mkdir build && cd build  
cmake ..  
make   
sudo make install   
#you can check cmake version by pressing  the instructions below
cmake --version  
```
**glog**
```shell
git clone https://github.com/google/glog.git 
mkdir build && cd build  
cmake ..  
make   
sudo make install   
```
**yaml-cpp (0.6 or higher is required)**
```
git clone -b yaml-cpp-0.6.0 https://github.com/jbeder/yaml-cpp.git  
cmake ..  
make   
sudo make install   
```  
**ros_qt5**
```shell
sudo apt-get install ros-melodic-qt-create    
sudo apt-get install ros-melodic-qt-build  
```
**gtsam 4.0**
``` 
  sudo add-apt-repository ppa:borglab/gtsam-release-4.0  
  sudo apt install libgtsam-dev libgtsam-unstable-dev  
```
## 2.硬件设备
**GNSS:** Ublox zedf9p      
**IMU:** FDISYSTEM N100       
**Lidar:** Velodyne 16     
**Camera:** ZED2i (Optional)


<left class="half">
    <img src="./pic/device.jpg" width=40% >
</left>


## 3.代码编译
```
cd $glins
catkin_make 
```
## 4.参数配置

## 5.数据集
### 5.1 公开数集  
[UrbanNav-HK-Data20190428](https://github.com/weisongwen/UrbanNavDataset)    
[M2DGR](https://github.com/SJTU-ViSYS/M2DGR)  

### 5.2 自采数据集
|  序列号  | 采集时间 | 大小 |  时长 | 特点 | rosbag |
| ----------- | ----------- |----------- | ----------- |----------- |----------- | 
| Camups_01      | 2022_11_26    | 7.9G | 120s  | Circle | [Aliyun](www.aliyundrive.com)


## 6.运行
```
chmod +x glins.sh
./test.sh
```

## 7.致谢    
 GVINIS  
IC-GVINS  
 LIO-SAM    
FAST LIO2  

## 8.许可证
遵循 GPLv3.

## 9.更新日志
2022/09/16 Tag v1.0  1.构建简单框架 &nbsp;  2.可视化组合惯导信息  
2022/10/02 Tag v2.0  1.使用ndt配合子图前端里程计  
2022/11/07 Tag v3.0  1.时间同步、畸变校正 &nbsp;2.使用g2o松耦合gnss  
2022/11/14 Tag v4.0  1.使用简单的NDT配合子地图进行定位  
todo 框架重构



