cmake_minimum_required(VERSION 3.21)
project(amber_gui_6_node)

set(CMAKE_CXX_STANDARD 17)

add_executable(amber_gui_6_node main.cpp udp.c udp.h huaxian.cpp huaxian.h)
