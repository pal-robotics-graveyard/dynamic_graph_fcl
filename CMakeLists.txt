#
#  Copyright (c) 2013, PAL Robotics, S.L. Author: Gennaro Raiola, Karsten Knese
#  Copyright 2010 CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

project(dynamic_graph_fcl)

string(REPLACE "-Woverloaded-virtual" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "-Werror=overloaded-virtual" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "-Werror=conversion-null" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "-Wconversion-null" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS geometric_shapes urdf tf dynamic-graph dynamic-graph-python sot-core sot_cmake fcl_capsule)
find_package(PkgConfig REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES dynamic_graph_fcl
	CATKIN_DEPENDS geometric_shapes urdf tf dynamic-graph dynamic-graph-python sot-core sot_cmake fcl_capsule
)

message("include dirs of dynamicgraphfcl:::::::: ${catkin_INCLUDE_DIRS}")

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(dynamic_graph_fcl
	src/DynamicGraphFCL.cpp src/URDFParser.cpp
        src/TFBroadcaster.cpp include/dynamic_graph_fcl/TFBroadcaster.h
        src/SOTCompensator.cpp include/dynamic_graph_fcl/SOTCompensator.h
	include/dynamic_graph_fcl/DynamicGraphFCL.h include/dynamic_graph_fcl/URDFParser.h
        include/dynamic_graph_fcl/Conversions.h include/dynamic_graph_fcl/SignalHelper.h
)
#link_directories(${catkin_LIBRARY_DIRS})
message("bnlaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa ${catkin_LIBRARIES}")
target_link_libraries(dynamic_graph_fcl ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(dynamic_graph_test test/test.cpp)

## Specify libraries to link a library or executablce target against
target_link_libraries(dynamic_graph_test
   ${catkin_LIBRARIES} dynamic_graph_fcl
)

# ADD SUBDIRECTORY FOR PYTHON WRAPPER EXPORT
add_subdirectory(src)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


