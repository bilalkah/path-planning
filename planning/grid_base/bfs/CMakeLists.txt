cmake_minimum_required(VERSION 3.15)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

add_library(
    bfs SHARED
    bfs.cpp
    bfs.h
    ${PROJECT_SOURCE_DIR}/planning/i_planning.h
    ${PROJECT_SOURCE_DIR}/planning/grid_base/common_grid_base.h
)

