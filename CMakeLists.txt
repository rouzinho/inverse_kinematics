# Copyright: 2012 iCub Facility, Istituto Italiano di Tecnologia
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

cmake_minimum_required(VERSION 3.5)
project(inverse_kinematics)

find_package(YARP)

#add_library(ctrl CtrlThread.cpp CtrlModule.cpp)
add_executable(cartesian_interface CtrlThread.cpp CtrlModule.cpp cartesian_interface.cpp)
target_compile_definitions(cartesian_interface PRIVATE _USE_MATH_DEFINES)
target_link_libraries(cartesian_interface ${YARP_LIBRARIES})

#add_executable(tutorial_gaze_interface tutorial_gaze_interface.cpp)
#target_compile_definitions(tutorial_gaze_interface PRIVATE _USE_MATH_DEFINES)
#target_link_libraries(tutorial_gaze_interface ${YARP_LIBRARIES})
