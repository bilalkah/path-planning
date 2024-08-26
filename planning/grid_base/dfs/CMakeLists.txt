add_library(
    dfs 
    SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/dfs.cpp
)

target_include_directories(
    dfs
    PUBLIC
    ${PROJECT_SOURCE_DIR}/planning/utility
)

target_link_libraries(
    dfs
    PUBLIC
    common_grid_base
)
