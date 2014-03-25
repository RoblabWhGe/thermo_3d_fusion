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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_boost_directories()

rosbuild_add_library(${PROJECT_NAME} 
	src/fair/core/io/IDeviceAdapter.h
	src/fair/devices/servo/CEposVMC.cpp
	src/fair/core/io/socket/CSocketAdapter.h
 	src/fair/core/io/socket/CSocketAdapter.cpp
	src/fair/core/io/com/CComAdapter.cpp
	src/fair/devices/factory/IDevice.h
	src/fair/core/base/CTimer.cpp
	src/fair/core/base/CTimer.h
	src/fair/core/base/common.h
	src/fair/devices/scanner/IScanner.h
	src/fair/devices/scanner/CSickLms100Scanner.cpp
	src/fair/devices/scanner/CSickLms100Scanner.h
	src/fair/devices/scanner/CSickScanner.cpp
	src/fair/devices/scanner/CSickScanner.h
	src/fair/devices/scanner/sickmsg.h
	src/data_structures/RingBuffer.cpp
	)


rosbuild_add_executable(${PROJECT_NAME}_rotary_drive src/rotary_drive.cpp)
rosbuild_add_executable(${PROJECT_NAME}_scan2cloud src/scan2cloud.cpp)
rosbuild_add_executable(${PROJECT_NAME}_cloud2PCD src/cloud2PCD.cpp)

rosbuild_link_boost(${PROJECT_NAME}_scan2cloud signals)
#target_link_libraries(${PROJECT_NAME}_scan2cloud ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_rotary_drive ${PROJECT_NAME})
