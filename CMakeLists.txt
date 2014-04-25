#
#  Copyright (c) 2013, PAL Robotics, S.L. Author: Gennaro Raiola, Karsten Knese
#  Copyright 2010 CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

project(dynamic_graph_fcl)

find_package(catkin REQUIRED COMPONENTS geometric_shapes urdf tf dynamic-graph dynamic-graph-python sot-core sot_cmake)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FCL REQUIRED fcl)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES dynamic_graph_fcl
	CATKIN_DEPENDS geometric_shapes urdf tf dynamic-graph dynamic-graph-python sot-core sot_cmake
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(dynamic_graph_fcl
	src/DynamicGraphFCL.cpp src/URDFParser.cpp
        src/TFBroadcaster.cpp include/dynamic_graph_fcl/TFBroadcaster.h
        src/SOTCompensator.cpp include/dynamic_graph_fcl/SOTCompensator.h
	include/dynamic_graph_fcl/DynamicGraphFCL.h include/dynamic_graph_fcl/URDFParser.h
        include/dynamic_graph_fcl/Conversions.h include/dynamic_graph_fcl/SignalHelper.h
)

## Declare a cpp executable
add_executable(dynamic_graph_test test/test.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dynamic_graph_test
   ${catkin_LIBRARIES} ${FCL_LIBRARIES} dynamic_graph_fcl
)
message("FCL_LIBRARIES IN DYNAMIC_GRAPHFCL:::  ${FCL_LIBRARIES}")
target_link_libraries(dynamic_graph_fcl ${catkin_LIBRARIES} ${FCL_LIBRARIES})

# ADD SUBDIRECTORY FOR PYTHON WRAPPER EXPORT
add_subdirectory(src)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


