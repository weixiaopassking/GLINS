# GLINS
一种低成本 融合GNSS、Lidar、IMU的长期建图与定位方案
  
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
     
## 1.依赖项
**ros melodic**
```shell
#just follow ros wiki for melodic version
```
**cmake  (3.16 or higher is required)**  
```
#You don't need to remove the old version, just press the instructions below
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
## 2.Device and config  
**GNSS:** Ublox zedf9p      
**IMU:** FDISYSTEM N100       
**Lidar:** Velodyne 16     
**Camera:** ZED2i (Optional)


<left class="half">
    <img src="./pic/device.jpg" width=40% >
</left>


## 3.Build 
```
cd $multisensor_fusion_localization_study
catkin_make 
```
## 4.Config Param
before run on public dataset or with your device, param must be seted.

## 5.Datasets
### 5.1 Public  
[UrbanNav-HK-Data20190428](https://github.com/weisongwen/UrbanNavDataset)    
[M2DGR](https://github.com/SJTU-ViSYS/M2DGR)  

### 5.2   Ours
|  Sequence   Name  | Collection Date | Total Size |  Duration | Features | Rosbag |
| ----------- | ----------- |----------- | ----------- |----------- |----------- | 
| Camups_01      | 2022_11_26    | 7.9G | 120s  | Circle | [Aliyun](www.aliyundrive.com)


## 6.Run
```
chmod +x glins.sh
./quick_start
```

## 7.Acknowledgements    
Thanks the following projects for the helps in developing and evaluating the repo.
 GVINIS  
IC-GVINS  
 LIO-SAM    
FAST LIO2  

## 9.Licence  
The source code is released under GPLv3 license.

## 10.Update Log   
2022/09/16 Tag v1.0  Combaine and display  imu and gnss data by rviz_satellite.    
2022/10/02 Tag v2.0  Use ndt  algorithm to calculatelidar odom.  
2022/11/07 Tag v3.0  Loosly couple  lidar  odom with gnss.  
2022/11/14 Tag v4.0  Simple use ndt algorithm to realize localization.  
To publish  Tag v5.0    



