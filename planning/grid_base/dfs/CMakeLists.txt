cmake_minimum_required(VERSION 3.22)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

set(
    DFS_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/dfs.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dfs.h
)

add_library(
    dfs SHARED
    ${DFS_FILES}
)

target_link_libraries(
    dfs
    PUBLIC
    common_grid_base
)
