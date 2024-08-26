add_library(
    astar 
    SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/astar.cpp
)

target_include_directories(
    astar
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/planning/utility
)

target_link_libraries(
    astar
    PUBLIC
    common_grid_base
)
