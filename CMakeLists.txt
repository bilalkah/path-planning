cmake_minimum_required(VERSION 3.22)  
project(PathFind)               
set(CMAKE_CXX_STANDARD 17)            
add_definitions("-Wall" "-g")


# include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/planning)
add_subdirectory(${PROJECT_SOURCE_DIR}/planning)

# enable test
enable_testing()
add_subdirectory(${PROJECT_SOURCE_DIR}/test)