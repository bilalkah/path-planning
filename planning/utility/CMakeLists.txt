add_library(
    common_planning
    SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/common_planning.cpp
)

target_include_directories(
    common_planning
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(
    common_grid_base
    SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/common_grid_base.cpp
)

target_include_directories(
    common_grid_base
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(
    common_tree_base
    SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/common_tree_base.cpp
)

target_include_directories(
    common_tree_base
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)