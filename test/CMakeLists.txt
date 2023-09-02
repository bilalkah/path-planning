include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

FetchContent_MakeAvailable(googletest)



add_executable(
    test_a_star
    test_a_star.cpp
    test_fixture.h
)
target_link_libraries(
    test_a_star
    a_star
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(test_a_star)