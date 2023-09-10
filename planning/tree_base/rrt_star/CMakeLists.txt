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
    RRT_STAR_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/rrt_star.cpp
)

add_library(
    rrt_star 
    SHARED
    ${RRT_STAR_FILES}
)
