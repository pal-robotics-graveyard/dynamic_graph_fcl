#
#  Copyright (c) 2013, PAL Robotics, S.L. Author: Gennaro Raiola, Karsten Knese
#  Copyright 2010 CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

project(dynamic_graph_fcl)

find_package(catkin REQUIRED COMPONENTS)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FCL REQUIRED fcl)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES dynamic_graph_fcl
	DEPENDS dynamic-graph dynamic-graph-python
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(dynamic_graph_fcl
	src/DynamicGraphFCL.cpp
	include/dynamic_graph_fcl/DynamicGraphFCL.h
)

## Declare a cpp executable
add_executable(dynamic_graph_test test/test.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dynamic_graph_test
   ${catkin_LIBRARIES} ${FCL_LIBRARIES} dynamic_graph_fcl
)
target_link_libraries(dynamic_graph_fcl ${catkin_LIBRARIES} ${FCL_LIBRARIES})
set(PKG_DEP dynamic-graph dynamic-graph-python)
foreach(DEP ${PKG_DEP})
	message("invoking dependency check for: ${DEP}")
	pkg_check_modules(${DEP} REQUIRED ${DEP})
	message("use dependency for:  ${${DEP}_LDFLAGS}")
	target_link_libraries(dynamic_graph_fcl ${${DEP}_LDFLAGS})
        target_link_libraries(dynamic_graph_test ${${DEP}_LDFLAGS})
endforeach(DEP)


# ADD SUBDIRECTORY FOR PYTHON WRAPPER EXPORT
add_subdirectory(src)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


