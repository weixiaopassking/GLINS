list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# [glog] sudo apt-get install libgoogle-glog-dev
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

# [eigen3] sudo apt-get install libeigen3-dev
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# sophus
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})


  set(third_party_libs glog gflags   ${catkin_LIBRARIES})