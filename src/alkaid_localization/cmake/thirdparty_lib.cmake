find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
list(APPEND thirdpart_LIBRARIES ${PCL_LIBRARIES} )
message("PCL载入成功")