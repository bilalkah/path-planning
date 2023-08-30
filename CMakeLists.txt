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

set(
    ASTAR
    ${PROJECT_SOURCE_DIR}/include/a_star.h
    ${PROJECT_SOURCE_DIR}/src/a_star.cpp
)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
    test_a_star
    tests/test_a_star.cpp
    ${HEADERS}
    ${SOURCES}
)
target_link_libraries(
    test_a_star
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(test_a_star)