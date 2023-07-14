list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#getst  https://stackoverflow.com/questions/24295876/cmake-cannot-find-googletest-required-library-in-ubuntu
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})



  set(thirdparty_libs  
${OpenCV_LIBS}
   ${PCL_LIBRARIES} 
${GTEST_BOTH_LIBRARIES} 
pthread
)