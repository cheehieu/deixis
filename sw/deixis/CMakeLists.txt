cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

FIND_PACKAGE( FLTK REQUIRED )
INCLUDE_DIRECTORIES(${FLTK_INCLUDE_DIR})
LINK_LIBRARIES(${FLTK_LIBRARIES})

rosbuild_init()

#rospack(deixis)
rosbuild_add_executable(deixis main2.cpp angles.cpp logger.h mutex.cpp pose.cpp random.cpp thread.cpp time.h utils.h vector.h wiimote.h)
rosbuild_add_executable(record_points record_points.cpp)
target_link_libraries(deixis cwiid pthread)

