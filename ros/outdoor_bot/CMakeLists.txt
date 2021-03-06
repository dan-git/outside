cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#needed for action client
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

rosbuild_add_library(rcm src/rcmRadar/rcm.c src/rcmRadar/rcmIf.c)

rosbuild_add_executable(robotPose_node nodes/robotPose_node.cpp)
rosbuild_add_executable(serialComm_node nodes/serialComm_node.cpp)
rosbuild_add_executable(movement_node nodes/movement_node.cpp)
rosbuild_add_executable(commandsController_node nodes/commandsController_node.cpp)
rosbuild_add_executable(patrol_node nodes/patrol_node.cpp)
rosbuild_add_executable(baseController_node nodes/baseController_node.cpp)
rosbuild_add_executable(sensors_node nodes/sensors_node.cpp)
rosbuild_add_executable(pololu_motor_control_node nodes/pololu_motor_control_node.cpp)
rosbuild_add_executable(pololu_motor_control nodes/pololu_motor_control.cpp)
rosbuild_add_executable(digcams nodes/digcams.cpp)
rosbuild_add_executable(webcams nodes/webcams.cpp)
rosbuild_add_executable(image_cvproc nodes/image_cvproc.cpp)
rosbuild_add_executable(ceiling_tracking_node nodes/ceiling_tracking_node.cpp)
rosbuild_add_executable(NavTargets nodes/NavTargets.cpp)
rosbuild_add_executable(mainTargets nodes/mainTargets.cpp)
rosbuild_add_executable(autonomous_node nodes/autonomous_node.cpp)
rosbuild_add_executable(image_converter src/image_converter.cpp)
#rosbuild_add_executable(simple_blobs src/simple_blobs.cpp)
#rosbuild_add_executable(blob_detector src/blob_detector.cpp)
#rosbuild_add_executable(radar src/rcmRadar/rcm.c src/rcmRadar/rcmIf.c src/rcmRadar/rcmSampleApp.c)
rosbuild_add_executable(radar src/rcmRadar/rcmSampleApp.c)

target_link_libraries(digcams ${OpenCV_LIBRARIES})
target_link_libraries(webcams ${OpenCV_LIBRARIES})
target_link_libraries(image_cvproc ${OpenCV_LIBRARIES})
target_link_libraries(image_converter ${OpenCV_LIBRARIES})
target_link_libraries(ceiling_tracking_node ${OpenCV_LIBRARIES})
target_link_libraries(NavTargets ${OpenCV_LIBRARIES})
target_link_libraries(mainTargets ${OpenCV_LIBRARIES})
target_link_libraries(autonomous_node ${OpenCV_LIBRARIES})
#target_link_libraries(simple_blobs ${OpenCV_LIBRARIES})
#target_link_libraries(blob_detector ${OpenCV_LIBRARIES})
target_link_libraries(digcams gphoto2 gphoto2_port)
target_link_libraries(radar rcm)

#rosbuild_add_executable(odomTester_node nodes/odomTester_node.cpp)

