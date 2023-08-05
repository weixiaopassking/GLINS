#pcl (ros melodic self-contained)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
list(APPEND thirdpart_LIBRARIES ${PCL_LIBRARIES} )
message("Load pcl successfully")

#GeographicLib  ( geographiclib.sourceforge.io cmake3.13.0)
find_package (GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})
list(APPEND thirdpart_LIBRARIES ${GeographicLib_LIBRARIES})
message("Load  geographicLib  successfully")

#Gtest (sudo apt-get install libgtest-dev)
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
list(APPEND thirdpart_LIBRARIES ${GTEST_BOTH_LIBRARIES} pthread)
message("Load  gtest  successfully")

#yaml 0.7.0
find_package (yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})
list(APPEND thirdpart_LIBRARIES ${YAML_CPP_LIBRARIES})
message("Load  yamlcpp  successfully")


#opencv 
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND thirdpart_LIBRARIES ${OpenCV_LIBS} )
message("Load  opencv  successfully")


# eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
#list(APPEND thirdpart_LIBRARIES ) need't
message("Load  Eigen  successfully")



# find_package (glog 0.6.0 REQUIRED)
# list(APPEND THIRD_PART_LIBRARIES glog::glog)

#g2o
# find_package( g2o REQUIRED )
# include_directories( ${G2O_INCLUDE_DIRS} )
# set(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )
# list(APPEND THIRD_PART_LIBRARIES ${G2O_LIBS})