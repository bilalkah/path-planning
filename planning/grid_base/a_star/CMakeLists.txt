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
    A_STAR_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/a_star.cpp
)

add_library(
    a_star 
    SHARED
    ${A_STAR_FILES}
)

target_link_libraries(
    a_star
    PUBLIC
    common_grid_base
)
