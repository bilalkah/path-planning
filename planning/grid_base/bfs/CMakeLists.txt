add_library(
    bfs 
    SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/bfs.cpp
)

target_include_directories(
    bfs
    PUBLIC
    ${PROJECT_SOURCE_DIR}/planning/utility
)

target_link_libraries(
    bfs
    PUBLIC
    common_grid_base
)
