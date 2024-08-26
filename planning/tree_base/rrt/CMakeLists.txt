add_library(
    rrt 
    SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/rrt.cpp
)

target_include_directories(
    rrt
    PUBLIC
    ${PROJECT_SOURCE_DIR}/planning/utility
)

target_link_directories(
    rrt
    PUBLIC
    ${PROJECT_SOURCE_DIR}/planning/utility
)

target_link_libraries(
    rrt
    PUBLIC
    common_tree_base
)
