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
    RRT_TREE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/rrt_tree.cpp
)

add_library(
    rrt_tree
    SHARED
    ${RRT_TREE_FILES}
)

target_link_libraries(
    rrt_tree
    PUBLIC
    common_tree_base
)
