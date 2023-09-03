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
    BFS_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/bfs.cpp
)

add_library(
    bfs SHARED
    ${BFS_FILES}
)

target_link_libraries(
    bfs
    PUBLIC
    common_grid_base
)