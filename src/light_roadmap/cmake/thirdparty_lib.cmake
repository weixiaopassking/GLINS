#pcl lib
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${PCL_LIBRARIES} )
message("Load pcl successfully") 
                   
#yaml-cpp
find_package (yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${YAML_CPP_LIBRARIES})
message("Load  yaml-cpp  successfully")