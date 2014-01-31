#
#  Copyright (c) 2013, PAL Robotics, S.L. Author: Gennaro Raiola, Karsten Knese
#  Copyright 2010 CNRS
#

# LAAS cmake include for python 
include(../cmake/python.cmake)
FINDPYTHON()
set(LIBRARY_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/lib)

set(WRAP_DIR ${LIBRARY_OUTPUT_PATH}/dynamic_graph/${PROJECT_NAME})
message("creating wrapper dir for ${PROJECT_NAME}: ${WRAP_DIR}")
message("python sitedir : ${PYTHON_SITELIB}")
file(MAKE_DIRECTORY "${LIBRARY_OUTPUT_PATH}/dynamic_graph/${PROJECT_NAME}")

set(NEW_ENTITY_CLASS
DynamicGraphFCL
)

dynamic_graph_python_module("${PROJECT_NAME}"
    ${PROJECT_NAME}
    ${PROJECT_NAME}/wrap
)

#PYTHON_INSTALL("dynamic_graph/${PROJECT_NAME}" "__init__.py" "${PYTHON_SITELIB}")
