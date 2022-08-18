cmake_minimum_required(VERSION 3.21)
project(amber_gui_5_node)
include_directories("/usr/local/include/eigen3/")
set(CMAKE_CXX_STANDARD 17)

add_executable(amber_gui_5_node main.cpp udp.c udp.h rrt.cpp rrt.h InverseKinematics.c InverseKinematics.h InverseKinematics_types.h rt_defines.h rt_nonfinite.h rt_nonfinite.c rtwtypes.h ForwardKinematics.h ForwardKinematics.c)
