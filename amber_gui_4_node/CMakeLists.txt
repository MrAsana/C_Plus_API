cmake_minimum_required(VERSION 3.21)
project(amber_gui_4_node)

set(CMAKE_CXX_STANDARD 17)

add_executable(amber_gui_4_node main.cpp udp.h udp.c)
