find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_mc_unit marching_cubes_tests.cpp)
target_link_libraries(${PROJECT_NAME}_mc_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME}_frontend ${PROJECT_NAME}_marching_cubes)
add_gtest_discover_tests(${PROJECT_NAME}_mc_unit)
add_dependencies(${PROJECT_NAME}_mc_unit ${PROJECT_NAME}_frontend ${PROJECT_NAME}_marching_cubes)
add_dependencies(run_tests ${PROJECT_NAME}_mc_unit)

# Install unit test executables
install(TARGETS ${PROJECT_NAME}_mc_unit
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
