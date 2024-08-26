add_library(
    rrt_star
    SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/rrt_star.cpp
)

target_include_directories(
    rrt_star
    PUBLIC
    ${PROJECT_SOURCE_DIR}/planning/utility
)

target_link_libraries(
    rrt_star
    PUBLIC
    common_tree_base
)
