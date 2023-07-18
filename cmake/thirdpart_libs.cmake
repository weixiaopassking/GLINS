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
#tbb   for ubuntu18.04 20.04应该能配置简单点
set(TBB_ROOT_DIR ${PROJECT_SOURCE_DIR}/thirdpart/tbb/oneTBB-2019_U8/oneTBB-2019_U8)
set(TBB_BUILD_DIR "tbb_build_dir=${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
set(TBB_BUILD_PREFIX "tbb_build_prefix=tbb")

include(${TBB_ROOT_DIR}/cmake/TBBBuild.cmake)
    tbb_build(TBB_ROOT ${TBB_ROOT_DIR}
            compiler=gcc-9
            stdver=c++17
            ${TBB_BUILD_DIR}
            ${TBB_BUILD_PREFIX}
            CONFIG_DIR
            TBB_DIR)

    find_package(TBB REQUIRED)

    include_directories(${PROJECT_SOURCE_DIR}/thirdpart/tbb/oneTBB-2019_U8/oneTBB-2019_U8/include)
    link_directories(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/tbb_release)


    #
set(thirdparty_libs  
${OpenCV_LIBS}
   ${PCL_LIBRARIES} 
${GTEST_BOTH_LIBRARIES} 
pthread
TBB::tbb
)