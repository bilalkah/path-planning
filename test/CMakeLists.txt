include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# # For Windows: Prevent overriding the parent project's compiler/linker settings
# set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(
    test_map
    test_map.cpp
    test_fixture.h
)

add_executable(
    test_a_star
    test_a_star.cpp
    test_fixture.h
)

add_executable(
    test_bfs
    test_bfs.cpp
    test_fixture.h
)

target_link_libraries(
    test_map
    GTest::gtest_main
)

target_link_libraries(
    test_a_star
    a_star
    GTest::gtest_main
)

target_link_libraries(
    test_bfs
    bfs
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(test_map)
gtest_discover_tests(test_a_star)
gtest_discover_tests(test_bfs)
