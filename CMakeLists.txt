cmake_minimum_required(VERSION 3.5...3.10)
project(PathFind VERSION 1.0 LANGUAGES CXX)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (WIN32)
    # use MSVC flags like /W3 or /W4
    # add_compile_options(/W3 /Zi /MDd)  # or whatever you need for debug symbols in MSVC
else()
    # add_compile_options(-Wall -Wextra -pedantic -Werror -g Wno-dev)  # or whatever you need for debug symbols in GCC/Clang
    # find_package(PkgConfig REQUIRED)
endif()


set(DATA_DIR ${PROJECT_SOURCE_DIR}/maps)
set(CONFIG_DIR ${PROJECT_SOURCE_DIR}/config)
add_definitions(
    -DDATA_DIR="${DATA_DIR}"
    -DCONFIG_DIR="${CONFIG_DIR}"
)

# ----------------------------------------- SDL ------------------------------------------
include(FetchContent)

FetchContent_Declare(
  SDL3
  GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
  GIT_TAG release-3.2.10
)

set(SDL3_STATIC OFF)
set(SDL3_SHARED ON)      # <<< Important
set(BUILD_SHARED_LIBS ON) # <<< Important

FetchContent_MakeAvailable(SDL3)

# Find the required packages and libraries
# ----------------------------------------- GIT ------------------------------------------
find_package(Git REQUIRED)
find_package(Threads REQUIRED)

if(GIT_FOUND)
    message(STATUS "Found GIT: ${GIT_EXECUTABLE}")
else()
    message(FATAL_ERROR "GIT not found, submodules will not be updated")
endif()

if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

add_subdirectory(${PROJECT_SOURCE_DIR}/third-party/yaml-cpp)

include_directories(
    ${SDL3_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/planning/
    ${PROJECT_SOURCE_DIR}/planning/utility
    ${PROJECT_SOURCE_DIR}/planning/tree_base/rrt
    ${PROJECT_SOURCE_DIR}/planning/tree_base/rrt_star
    ${PROJECT_SOURCE_DIR}/planning/grid_base/astar
    ${PROJECT_SOURCE_DIR}/planning/grid_base/bfs
    ${PROJECT_SOURCE_DIR}/planning/grid_base/dfs
    ${PROJECT_SOURCE_DIR}/third-party/yaml-cpp/include
    ${PROJECT_SOURCE_DIR}/tools/visualizer
)

# ----------------------------------------- utility ------------------------------------------
file(
    GLOB_RECURSE 
    UTILITY_SRC 
    ${PROJECT_SOURCE_DIR}/planning/utility/*
)

add_library(
    planner_utility 
    STATIC
    ${UTILITY_SRC}
)

# ----------------------------------------- A* ------------------------------------------
file(
    GLOB_RECURSE 
    ASTAR_SRC 
    ${PROJECT_SOURCE_DIR}/planning/grid_base/astar/*
)

add_library(
    astar 
    STATIC
    ${ASTAR_SRC}
)

target_link_libraries(
    astar 
    PRIVATE
    planner_utility
)

# ----------------------------------------- BFS ------------------------------------------
file(
    GLOB_RECURSE 
    BFS_SRC
    ${PROJECT_SOURCE_DIR}/planning/grid_base/bfs/*
)


add_library(
    bfs 
    STATIC
    ${BFS_SRC}
)

target_link_libraries(
    bfs 
    PRIVATE
    planner_utility
)
# ----------------------------------------- DFS ------------------------------------------ 
file(
    GLOB_RECURSE 
    DFS_SRC
    ${PROJECT_SOURCE_DIR}/planning/grid_base/dfs/*
)

add_library(
    dfs 
    STATIC
    ${DFS_SRC}
)

target_link_libraries(
    dfs 
    PRIVATE
    planner_utility
)
# ----------------------------------------- RRT ------------------------------------------
file(
    GLOB_RECURSE 
    RRT_SRC 
    ${PROJECT_SOURCE_DIR}/planning/tree_base/rrt/*
)

add_library(
    rrt 
    STATIC
    ${RRT_SRC}
)

target_link_libraries(
    rrt 
    PRIVATE
    planner_utility
)
# ----------------------------------------- RRT* ------------------------------------------
file(
    GLOB_RECURSE 
    RRT_STAR_SRC 
    ${PROJECT_SOURCE_DIR}/planning/tree_base/rrt_star/*
)

add_library(
    rrt_star 
    STATIC
    ${RRT_STAR_SRC}
)

target_link_libraries(
    rrt_star 
    PRIVATE
    planner_utility
)
# ----------------------------------------- visualizer ------------------------------------------
file(
    GLOB_RECURSE 
    VIZ_SRC 
    ${PROJECT_SOURCE_DIR}/tools/visualizer/*
)

add_library(
    visualizer
    STATIC
    ${VIZ_SRC}
)

target_link_libraries(
    visualizer
    PRIVATE
    planner_utility
    SDL3::SDL3
)

if (WIN32)
    add_custom_command(TARGET visualizer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:SDL3::SDL3>
        $<TARGET_FILE_DIR:visualizer>
    )
endif()

# ----------------------------------------- main ------------------------------------------
add_executable(
    main 
    main.cpp
    ${UTILITY_SRC}
)

target_link_libraries(main astar bfs dfs rrt rrt_star visualizer yaml-cpp planner_utility)

if (WIN32)
    add_custom_command(TARGET main POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL3::SDL3>
            $<TARGET_FILE_DIR:main>
    )

    add_custom_command(TARGET main POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:yaml-cpp>
            $<TARGET_FILE_DIR:main>
    )
endif()

# ----------------------------------------- test ------------------------------------------

if (NOT WIN32)
    enable_testing()
    include(GoogleTest)
    FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    # set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    set(TARGET_LIST 
        test_map
        test_a_star
        test_bfs
        test_dfs
        test_access_data
        test_rrt_star
        test_rrt
        test_ray_cast
    )

    foreach(TARGET ${TARGET_LIST})
        add_executable(
            ${TARGET} 
            ${PROJECT_SOURCE_DIR}/test/${TARGET}.cpp 
            ${UTILITY_SRC}
        )
        target_link_libraries(${TARGET} GTest::gtest_main astar bfs dfs rrt_star rrt planner_utility)
        add_test(NAME ${TARGET} COMMAND ${TARGET})

        if (WIN32)
            add_custom_command(TARGET ${TARGET} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:SDL3::SDL3>
                    $<TARGET_FILE_DIR:${TARGET}>
            )
        endif()
    
    endforeach()

    foreach(TARGET ${TARGET_LIST})
        gtest_discover_tests(${TARGET})
    endforeach()
endif()