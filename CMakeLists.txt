cmake_minimum_required(VERSION 3.22)  
project(PathFind)               
set(CMAKE_CXX_STANDARD 17)            
add_definitions("-Wall" "-g")

find_package(SFML COMPONENTS window graphics system)
find_package(Git QUIET)

set(DATA_DIR ${PROJECT_SOURCE_DIR}/maps)
set(CONFIG_DIR ${PROJECT_SOURCE_DIR}/config)
add_definitions(
    -DDATA_DIR="${DATA_DIR}"
    -DCONFIG_DIR="${CONFIG_DIR}"
)

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

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/third-party/yaml-cpp/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/third-party/yaml-cpp/include
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/third-party/yaml-cpp/include
)


add_subdirectory(${PROJECT_SOURCE_DIR}/third-party/yaml-cpp)
add_subdirectory(${PROJECT_SOURCE_DIR}/planning)
add_subdirectory(${PROJECT_SOURCE_DIR}/tools)

# enable test
enable_testing()
add_subdirectory(${PROJECT_SOURCE_DIR}/test)

add_executable(
    main 
    main.cpp
)

target_link_libraries(main a_star bfs dfs rrt rrt_star visualizer yaml-cpp)
target_compile_features(main PRIVATE cxx_std_17)
