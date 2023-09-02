cmake_minimum_required(VERSION 3.15)

include_directories(
    ${CMAKE_BINARY_DIR}
    ../
    ../../
)



add_library(
    a_star SHARED
    a_star.cpp
    a_star.h
    ../../common.h
    ../common.h
)

