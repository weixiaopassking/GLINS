list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#pcl (ros melodic self-contained)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${PCL_LIBRARIES} )
message("Load pcl successfully")

#GeographicLib  ( geographiclib.sourceforge.io cmake3.13.0)
find_package (GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${GeographicLib_LIBRARIES})
message("Load  geographicLib  successfully")

#Gtest (sudo apt-get install libgtest-dev)
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${GTEST_BOTH_LIBRARIES} pthread)
message("Load  gtest  successfully")

#yaml 0.7.0
find_package (yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${YAML_CPP_LIBRARIES})
message("Load  yamlcpp  successfully")


#opencv 
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${OpenCV_LIBS} )
message("Load  opencv  successfully")


# eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
#list(APPEND thirdpart_LIBRARIES ) need't
message("Load  Eigen  successfully")


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

set(TBB_ROOT_DIR ${alkaid_localization_path}/thirdparty/tbb/oneTBB-2019_U8/oneTBB-2019_U8)
set(TBB_BUILD_DIR "tbb_build_dir=${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
set(TBB_BUILD_PREFIX "tbb_build_prefix=tbb")

extract_file(${alkaid_localization_path}/thirdparty/tbb/2019_U8.tar.gz  $${alkaid_localization_path}/thirdparty/tbb/oneTBB-2019_U8)

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

include_directories(${alkaid_localization_path}/thirdparty/tbb/oneTBB-2019_U8/oneTBB-2019_U8/include)
link_directories(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/tbb_release)

list(APPEND thirdparty_LIBRARIES   TBB::tbb)

#sophus (from pkg file)
include_directories(${alkaid_localization_path}/thirdparty/sophus)
message("Load  sophus  successfully")

# find_package (glog 0.6.0 REQUIRED)
# list(APPEND THIRD_PART_LIBRARIES glog::glog)

#g2o
# find_package( g2o REQUIRED )
# include_directories( ${G2O_INCLUDE_DIRS} )
# set(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )
# list(APPEND THIRD_PART_LIBRARIES ${G2O_LIBS})