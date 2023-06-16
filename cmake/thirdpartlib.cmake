list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# [glog] sudo apt-get install libgoogle-glog-dev
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

#[ros]


#[yamlcpp]




  set(third_party_libs glog   ${catkin_LIBRARIES})