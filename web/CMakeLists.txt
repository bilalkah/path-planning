# WebAssembly build of the planners for the project page in site/.
# Configure with emcmake. web/build.sh runs the whole build.

add_executable(
    path_planning_web
    ${CMAKE_CURRENT_SOURCE_DIR}/planner_bindings.cpp
)

target_include_directories(
    path_planning_web
    PRIVATE
    ${PROJECT_SOURCE_DIR}/planning
)

target_link_libraries(path_planning_web PRIVATE astar bfs dfs rrt rrt_star)
target_compile_features(path_planning_web PRIVATE cxx_std_17)

# Emits path_planning.js and path_planning.wasm. The page loads them in a
# Web Worker with createPathPlanning().
set_target_properties(
    path_planning_web
    PROPERTIES
    OUTPUT_NAME path_planning
    SUFFIX ".js"
)

target_link_options(
    path_planning_web
    PRIVATE
    -lembind
    -sMODULARIZE=1
    -sEXPORT_NAME=createPathPlanning
    -sENVIRONMENT=web,worker
    -sFILESYSTEM=1
    -sALLOW_MEMORY_GROWTH=1
)
