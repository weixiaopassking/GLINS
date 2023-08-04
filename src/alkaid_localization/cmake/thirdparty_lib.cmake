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
