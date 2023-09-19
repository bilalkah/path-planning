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
    TREE_VISUALIZER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tree_visualizer.cpp
    ${COMMON_PLANNING_FILES}
)

add_library(
    tree_visualizer
    SHARED
    ${TREE_VISUALIZER_SOURCES}
)

target_link_libraries(
    tree_visualizer
    PUBLIC
    sfml-system 
    sfml-graphics 
    sfml-window
    common_tree_base
)

