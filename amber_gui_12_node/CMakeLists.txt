cmake_minimum_required(VERSION 3.21)
project(amber_gui_12_node)

set(CMAKE_CXX_STANDARD 14)

add_executable(amber_gui_12_node main.cpp udp.c udp.h)
