# multisensor_fusion_localization_study
This repo is just  a record for my study for multisensor_fusion_localization and  is released by.Tag form. 

**Contacts:** For any technical issues, please open an issue at this repository.


## 1.Dependency
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
## 2.Device and config  
**GNSS:** Ublox zedf9p      
**IMU:** FDISYSTEM N100       
**Lidar:** Velodyne 16     
**Camera:** ZED2i (Optional)

## 3.Build 
```
cd $multisensor_fusion_localization_study
catkin_make 
```
## 4.Config Param
before run on public dataset or with your device, param must be seted.



## 5.Run
```
chmod +x glins.sh
./glins.sh
```

## 6.Acknowledgements    



## 7.Update log 
2022/9/16 Tag v1.0  
2022/10/2 Tag v2.0  
2022/11/7 Tag v3.0  
2022/11/14 Tag v4.0   

## 8.Licence  
The source code is released under GPLv3 license.
