cmake_minimum_required(VERSION 3.22)  
project(PathFind)               
set(CMAKE_CXX_STANDARD 17)            
add_definitions("-Wall" "-g")


include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)


file(
    GLOB_RECURSE
    HEADERS
    ${PROJECT_SOURCE_DIR}/include/*.h
)

file(
    GLOB_RECURSE
    SOURCES
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)

