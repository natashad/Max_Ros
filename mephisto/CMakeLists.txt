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
#uncomment if you have defined services
rosbuild_gensrv()


#===============================================================================

set(HEIGHT_CONTROLLER_SOURCE_FILES         
	src/HeightControlNode.cpp
	src/Core.cpp
	src/HeightController.cpp
)

set(HEIGHT_CONTROLLER_HEADER_FILES
	src/Core.h
	src/HeightController.h
)

add_definitions( -Wall )

rosbuild_add_executable(height_controller ${HEIGHT_CONTROLLER_SOURCE_FILES} ${HEIGHT_CONTROLLER_HEADER_FILES})

set(SFML_LIBRARIES sfml-graphics sfml-window sfml-system sfml-audio)
target_link_libraries(height_controller m)

#===============================================================================

set(FORCE_ESTIMATOR_SOURCE_FILES         
	src/Core.cpp
	src/ForceEstimationNode.cpp
	src/ForceEstimator.cpp
)

set(FORCE_ESTIMATOR_HEADER_FILES
	src/Core.h
	src/ForceEstimator.h
)

add_definitions( -Wall )

rosbuild_add_executable(force_estimator ${FORCE_ESTIMATOR_SOURCE_FILES} ${FORCE_ESTIMATOR_HEADER_FILES})

target_link_libraries(force_estimator m)

#===============================================================================

set(REACTOR_SOURCE_FILES         
	src/Core.cpp
	src/State.cpp
	src/FloatState.cpp
	src/StateMachine.cpp
	src/ReactorNode.cpp
)

set(REACTOR_HEADER_FILES
	src/Core.h
	src/State.h
)


add_definitions( -Wall )

rosbuild_add_executable(reactor ${REACTOR_SOURCE_FILES} ${REACTOR_HEADER_FILES})

set(SFML_LIBRARIES sfml-graphics sfml-window sfml-system sfml-audio)
target_link_libraries(reactor m)

#===============================================================================

set(DRUM_MACHINE_SOURCE_FILES         
	src/Core.cpp
	src/State.cpp
	src/FloatState.cpp
	src/DrumMachine.cpp
	src/StateMachine.cpp
	src/DrumNode.cpp
)

set(DRUM_MACHINE_HEADER_FILES
	src/Core.h
	src/State.h
	src/DrumMachine.h
)


add_definitions( -Wall )

rosbuild_add_executable(drum ${DRUM_MACHINE_SOURCE_FILES} ${DRUM_MACHINE_HEADER_FILES})

set(SFML_LIBRARIES sfml-graphics sfml-window sfml-system sfml-audio)
target_link_libraries(reactor m)