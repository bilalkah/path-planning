cmake_minimum_required(VERSION 3.22)  
project(PathFind)               
set(CMAKE_CXX_STANDARD 17)            
add_definitions("-Wall" "-g")


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

add_subdirectory(${PROJECT_SOURCE_DIR}/planning)

# enable test
enable_testing()
add_subdirectory(${PROJECT_SOURCE_DIR}/test)

add_executable(main main.cpp)
target_link_libraries(main a_star bfs dfs)