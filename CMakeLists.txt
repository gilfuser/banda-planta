cmake_minimum_required(VERSION 3.21)
project(banda_planta)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories(src)

add_executable(banda_planta
        src/main.cpp)
