add_library(
    visualizer
    STATIC
    visualizer.cpp
)

target_include_directories(visualizer PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/third-party/SDL/include
    ${PROJECT_SOURCE_DIR}/planning
)

target_link_libraries(
    visualizer
    PUBLIC
    common_planning
    SDL3::SDL3
)

