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
)

add_subdirectory(grid_base/astar)
add_subdirectory(grid_base/bfs)
add_subdirectory(grid_base/dfs)
add_subdirectory(tree_base/rrt)
add_subdirectory(tree_base/rrt_star)
add_subdirectory(utility)