list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


include(cmake/colorful.cmake)
set(thirdparty_libs )

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
list(APPEND thirdparty_libs ${PCL_LIBRARIES} )
message("${BoldYellow}PCL is Ok${ColourReset}")
#————————————————————————————————————

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND thirdparty_libs ${OpenCV_LIBS} )
message("${BoldYellow}OpenCv is Ok${ColourReset}")
#————————————————————————————————————

#getst
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
list(APPEND thirdparty_libs  ${GTEST_BOTH_LIBRARIES}  pthread)
message("${BoldYellow}Gtest is Ok${ColourReset}")
#————————————————————————————————————

# yaml cpp
find_package (yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})
list(APPEND thirdparty_libs ${YAML_CPP_LIBRARIES})
message("${BoldYellow}Yamlcpp is Ok${ColourReset}")
#————————————————————————————————————

#tbb
set(TBB_ROOT_DIR ${PROJECT_SOURCE_DIR}/thirdpart/tbb/oneTBB-2019_U8/oneTBB-2019_U8)
set(TBB_BUILD_DIR "tbb_build_dir=${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
set(TBB_BUILD_PREFIX "tbb_build_prefix=tbb")
include(${TBB_ROOT_DIR}/cmake/TBBBuild.cmake)
tbb_build(TBB_ROOT ${TBB_ROOT_DIR}
            compiler=gcc-9
            stdver=c++
            ${TBB_BUILD_DIR}
            ${TBB_BUILD_PREFIX}
            CONFIG_DIR
            TBB_DIR)

find_package(TBB REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/thirdpart/tbb/oneTBB-2019_U8/oneTBB-2019_U8/include)
link_directories(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/tbb_release)

list(APPEND thirdparty_libs TBB::tbb )

message("${BoldYellow}TBB is Ok${ColourReset}")
#————————————————————————————————————


#ceres
#geolib
#ros

