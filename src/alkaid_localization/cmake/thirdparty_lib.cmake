#pcl (ros melodic self-contained)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
list(APPEND thirdpart_LIBRARIES ${PCL_LIBRARIES} )
message("PCL has been loaded")

#GeographicLib  ( geographiclib.sourceforge.io cmake3.13.0)
find_package (GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})
list(APPEND thirdpart_LIBRARIES ${GeographicLib_LIBRARIES})

