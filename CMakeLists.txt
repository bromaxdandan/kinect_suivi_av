cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_make_distribution(0.1.3)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include(FindPkgConfig)
pkg_check_modules(OPENNI libopenni)
#rosbuild_gensrv()
rosbuild_genmsg()
include_directories(${OPENNI_INCLUDEDIR} /usr/include/nite)
#rosbuild_add_executable(openni_tracker src/openni_tracker.cpp )
#rosbuild_add_executable(brahim_suivi src/brahim_suivi.cpp )
rosbuild_add_executable(brahim_suivi1 src/brahim_suivi1.cpp )
#rosbuild_add_executable(KinectController src/KinectController.cpp )
#rosbuild_add_executable(openni_tracker1 src/props_openni_tracker.cpp )
#rosbuild_add_executable(openni_tracker1 src/openni_tracker.cpp)
target_link_libraries(brahim_suivi ${OPENNI_LIBRARIES})
target_link_libraries(brahim_suivi1 ${OPENNI_LIBRARIES})

#include_directories(/usr/include/openni /usr/include/nite /usr/include/ni)

#target_link_libraries(skeleton_tracker glut OpenNI)
