  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")


#getst
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES  ${GTEST_BOTH_LIBRARIES}  pthread)
message("${BoldYellow}Gtest is Ok${ColourReset}")
#————————————————————————————————————

# yaml cpp
find_package (yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${YAML_CPP_LIBRARIES})
message("${BoldYellow}Yamlcpp is Ok${ColourReset}")
#————————————————————————————————————


#sophus 本地 thirdparty导入
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)
message("${BoldYellow}Sophus is Ok${ColourReset}")
#————————————————————————————————————

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${PCL_LIBRARIES} )
message("${BoldYellow}PCL is Ok${ColourReset}")
#————————————————————————————————————

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${OpenCV_LIBS} )
message("${BoldYellow}OpenCv is Ok${ColourReset}")
#————————————————————————————————————

#tbb  本地 thirdparty导入
function(extract_file filename extract_dir)
        message(STATUS "Extract ${filename} to ${extract_dir} ...")
        set(temp_dir ${extract_dir})
        if(EXISTS ${temp_dir})
            file(REMOVE_RECURSE ${temp_dir})
        endif()
        file(MAKE_DIRECTORY ${temp_dir})
        execute_process(COMMAND ${CMAKE_COMMAND} -E tar -xvzf ${filename}
                WORKING_DIRECTORY ${temp_dir})
endfunction()

set(TBB_ROOT_DIR ${project_path}/thirdparty/tbb/oneTBB-2019_U8/oneTBB-2019_U8)
set(TBB_BUILD_DIR "tbb_build_dir=${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
set(TBB_BUILD_PREFIX "tbb_build_prefix=tbb")

extract_file(${project_path}/thirdparty/tbb/2019_U8.tar.gz ${project_path}/thirdparty/tbb/oneTBB-2019_U8)

include(${TBB_ROOT_DIR}/cmake/TBBBuild.cmake)

    #message(STATUS "======TBB_BUILD_DIR = ${TBB_BUILD_DIR}")
    #message(STATUS "======TBB_BUILD_PREFIX = ${TBB_BUILD_PREFIX}")

tbb_build(TBB_ROOT ${TBB_ROOT_DIR}
            compiler=gcc-9
            stdver=c++17
            ${TBB_BUILD_DIR}
            ${TBB_BUILD_PREFIX}
            CONFIG_DIR
            TBB_DIR)

find_package(TBB REQUIRED)

include_directories(${project_path}/thirdparty/tbb/oneTBB-2019_U8/oneTBB-2019_U8/include)
link_directories(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/tbb_release)

list(APPEND thirdparty_LIBRARIES   TBB::tbb)
message("${BoldYellow}TBB is Ok${ColourReset}")
#————————————————————————————————————
