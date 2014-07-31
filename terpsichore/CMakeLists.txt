cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
## Generate messages in the 'msg' folder
#add_message_files(FILES pair.msg bardata.msg)
#uncomment if you have defined services
#rosbuild_gensrv()


# ------------------------------- Conductor ------------------------------------
# set header ans source files
set(CONDUCTOR_SOURCE_FILES
    src/ConductorNode.cpp       
    src/LiveReceiver.cpp
    src/Translator.cpp
)
set(CONDUCTOR_HEADER_FILES    
    src/LiveReceiver.h
    src/Translator.h
)

# set required libs and headers
include_directories(
)  

link_directories(
)
set(OSC_LIBRARIES oscpack)

# build!
rosbuild_add_executable(interpreter ${CONDUCTOR_SOURCE_FILES} ${CONDUCTOR_HEADER_FILES})
set(SFML_LIBRARIES sfml-graphics sfml-window sfml-system sfml-audio)
target_link_libraries(interpreter ${OSC_LIBRARIES} ${SFML_LIBRARIES} m)
add_definitions(-std=c++0x)
#add_dependencies(drone_stateestimation thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

# ------------------------------- OSC relay ------------------------------------
# set header ans source files
set(OSC_RELAY_SOURCE_FILES
    src/Relay.cpp
)
set(OSC_RELAY_HEADER_FILES
)

# set required libs and headers
include_directories(
)  

link_directories(
)
set(OSC_LIBRARIES oscpack)

# build!
rosbuild_add_executable(osc_relay ${OSC_RELAY_SOURCE_FILES} ${OSC_RELAY_HEADER_FILES})
target_link_libraries(osc_relay ${OSC_LIBRARIES} m)
add_definitions(-std=c++0x)
#add_dependencies(drone_stateestimation thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
