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
    RRT_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/rrt.cpp
)

add_library(
    rrt 
    SHARED
    ${RRT_FILES}
)

target_link_libraries(
    rrt
    PUBLIC
    common_tree_base
)
