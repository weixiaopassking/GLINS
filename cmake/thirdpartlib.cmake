list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})


  set(thirdparty_libs   ${PCL_LIBRARIES} )