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
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

execute_process(COMMAND qmake WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/qextserialport)
execute_process(COMMAND make WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/qextserialport)
execute_process(COMMAND mv ${PROJECT_SOURCE_DIR}/qextserialport/libqextserialport.so.1.2.0 ${LIBRARY_OUTPUT_PATH})
execute_process(COMMAND mv ${PROJECT_SOURCE_DIR}/qextserialport/libqextserialport.so.1.2 ${LIBRARY_OUTPUT_PATH})
execute_process(COMMAND mv ${PROJECT_SOURCE_DIR}/qextserialport/libqextserialport.so.1 ${LIBRARY_OUTPUT_PATH})
execute_process(COMMAND mv ${PROJECT_SOURCE_DIR}/qextserialport/libqextserialport.so ${LIBRARY_OUTPUT_PATH})

