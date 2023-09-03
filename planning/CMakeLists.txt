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
    COMMON_PLANNING_FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common_planning.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/common_planning.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/node_parent.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/data_types.h
)

add_subdirectory(grid_base)
add_subdirectory(tree_base)
