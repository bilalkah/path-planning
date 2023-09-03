cmake_minimum_required(VERSION 3.22)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}
)


add_library(
    common_grid_base SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common_grid_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common_grid_base.h
    ${COMMON_PLANNING_FILES}
)

add_subdirectory(a_star)
add_subdirectory(bfs)
add_subdirectory(dfs)