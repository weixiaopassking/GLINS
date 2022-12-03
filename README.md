# multisensor_fusion_localization_study
This repo is just  a record for my study.The project is still coding.

**Related video**:   
**Pipeline:**     
**Related papers:**   
**Contacts:** For any technical issues, please open an issue at this repository.


## 1.Dependency
**ros melodic**
```shell
#just follow ros wiki https://wiki.ros.org/melodic/Installation/Ubuntu  
```
**cmake  (3.16 or higher is required)**  
```
git clone -b v3.16.5  https://github.com/Kitware/CMake.git
mkdir build && cd build
cmake ..
make 
sudo make install 
```
**GeographicLib**
```
git clone https://github.com/geographiclib/geographiclib.git  
mkdir build && cd build  
cmake ..  
make   
sudo make install   
## you can check cmake version use following command  
cmake --version  
```
**glog**
```
git clone https://github.com/google/glog.git 
mkdir build && cd build  
cmake ..  
make   
sudo make install   
```
**yaml-cpp 0.6**
```
git clone -b yaml-cpp-0.6.0 https://github.com/jbeder/yaml-cpp.git  
cmake ..  
make   
sudo make install   
```  
**ros_qt5**
```
sudo apt-get install ros-melodic-qt-create    
sudo apt-get install ros-melodic-qt-build  
```
## 2.Device and config  
**GNSS:** Ublox zedf9p      
**IMU:** FDISYSTEM N100       
**Lidar:** Velodyne 16     
**Camera:** ZED2i (Opt)

## 3.Build

## 4.Run with dataset    
+ Public:   
1. UrbanNavDataset
+ Ours:  
## 5.Run with your device   

## 6.Acknowledgements    



## 7.Update log 

2022/11/13 Simple multisensor mapping and localization framework.  
2022/11/23 Sensors  has been tested and  drivers has been added into this repository.

## 8.Licence  
The source code is released under GPLv3 license.
