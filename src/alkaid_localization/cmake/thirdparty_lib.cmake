#pcl (ros melodic self-contained)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${PCL_LIBRARIES} )
message("Load pcl successfully")

#opencv   (ros melodic self-contained)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${OpenCV_LIBS} )
message("Load  opencv  successfully")


#GeographicLib  ( geographiclib.sourceforge.io cmake3.13.0 required)
find_package (GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${GeographicLib_LIBRARIES})
message("Load  geographicLib  successfully")

#Gtest (sudo apt-get install libgtest-dev)
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${GTEST_BOTH_LIBRARIES} pthread)
message("Load  gtest  successfully")

#yaml 0.7.0 from source 
find_package (yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})
list(APPEND thirdparty_LIBRARIES ${YAML_CPP_LIBRARIES})
message("Load  yamlcpp  successfully")


# eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
#list(APPEND thirdpart_LIBRARIES ) need't
message("Load  Eigen  successfully")



#tbb (use https://github.com/wjakob/tbb)
find_package(TBB REQUIRED)
list(APPEND thirdparty_LIBRARIES  TBB::tbb)
message("Load  TBB  successfully")


#sophus (from thirdparty file)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)
message("Load  sophus  successfully")

#g2o  (from thirdparty file)
include(${PROJECT_SOURCE_DIR}/cmake/g2o.cmake)
list(APPEND thirdparty_LIBRARIES  ${g2o_libs})
message("Load  g2o  successfully")