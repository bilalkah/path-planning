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

set(
    COMMOND_TREE_BASE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common_tree_base.cpp
)

add_library(
    common_tree_base
    SHARED
    ${COMMOND_TREE_BASE}
    ${COMMON_PLANNING_FILES}
)

add_subdirectory(rrt)


