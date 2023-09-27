#pcl库
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${PCL_LIBRARIES} )
message("Load pcl successfully") 
                   
#yaml库
