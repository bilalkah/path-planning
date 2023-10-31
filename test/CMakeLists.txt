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

set(TARGET_LIST 
    test_map
    test_a_star
    test_bfs
    test_dfs
    test_access_data
    test_rrt_star
    test_rrt
)

foreach(TARGET ${TARGET_LIST})
    add_executable(${TARGET} ${TARGET}.cpp)
    target_link_libraries(${TARGET} GTest::gtest_main common_grid_base a_star bfs dfs rrt_star rrt)
    add_test(NAME ${TARGET} COMMAND ${TARGET})
    
endforeach()

include(GoogleTest)

foreach(TARGET ${TARGET_LIST})
    gtest_discover_tests(${TARGET})
endforeach()
