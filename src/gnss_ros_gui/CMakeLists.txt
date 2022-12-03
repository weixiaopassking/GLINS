
cmake_minimum_required(VERSION 2.8.0)
project(gnss_ros_gui)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


find_package(catkin REQUIRED COMPONENTS qt_build roscpp)
include_directories(${catkin_INCLUDE_DIRS})

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package()


#rosbuild_prepare_qt4(QtCore QtGui) 


file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/gnss_ros_gui/*.hpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})



file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)


add_executable(gnss_ros_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(gnss_ros_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS gnss_ros_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

