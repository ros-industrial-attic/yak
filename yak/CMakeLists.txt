cmake_minimum_required(VERSION 3.10.0)
project(yak VERSION 0.2.0 LANGUAGES CXX)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED True)
endif()

if(NOT DEFINED CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 14)
  set(CMAKE_CUDA_STANDARD_REQUIRED True)
endif()

find_package(CUDA 9.0 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui)
find_package(PCL 1.8 REQUIRED COMPONENTS common io geometry)

find_package(OpenMP REQUIRED)
if(NOT TARGET OpenMP::OpenMP_CXX)
    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
endif()

find_package(Eigen3 REQUIRED NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
    find_package(Threads REQUIRED)
    add_library(Eigen3::Eigen IMPORTED INTERFACE)
    set_property(TARGET Eigen3::Eigen
                 PROPERTY INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
    set_property(TARGET Eigen3::Eigen
                 PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
endif()

# Macro to sanitize and separate a mixed list of options and definitions and add them to the specified target
macro(target_add_options_and_definitions TARGET)
  set(DEFINITIONS "${ARGN}")

  # Add a space before each prefix
  string(REPLACE "-D" " -D" DEFINITIONS ${DEFINITIONS})
  string(REPLACE "-m" " -m" DEFINITIONS ${DEFINITIONS})
  string(REPLACE "-f" " -f" DEFINITIONS ${DEFINITIONS})
  string(STRIP ${DEFINITIONS} DEFINITIONS)

  # Convert into semicolon-separated list
  separate_arguments(DEFS_SPLIT NATIVE_COMMAND ${DEFINITIONS})

  foreach(DEF IN LISTS DEFS_SPLIT)
    string(SUBSTRING "${DEF}" 0 2 DEF_PREFIX)
    if ("${DEF_PREFIX}" STREQUAL "-m" OR "${DEF_PREFIX}" STREQUAL "-f")
      target_compile_options(${TARGET} PUBLIC ${DEF})
    else()
      target_compile_definitions(${TARGET} PUBLIC ${DEF})
    endif()
  endforeach()
endmacro()

CUDA_SELECT_NVCC_ARCH_FLAGS(ARCH_FLAGS Auto)
LIST(APPEND CUDA_NVCC_FLAGS ${ARCH_FLAGS})
list(APPEND CUDA_NVCC_FLAGS "--compiler-options -fPIC")

# Core CUDA Library for depth image processing
cuda_add_library(${PROJECT_NAME}
  src/kfusion/core.cpp
  src/kfusion/device_memory.cpp
  src/kfusion/imgproc.cpp
  src/kfusion/kinfu.cpp
  src/kfusion/precomp.cpp
  src/kfusion/projective_icp.cpp
  src/kfusion/tsdf_volume.cpp
  src/cuda/imgproc.cu
  src/cuda/proj_icp.cu
  src/cuda/tsdf_volume.cu)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${CUDA_CUDA_LIBRARY}
  Eigen3::Eigen)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})

# Jmeyer - Create a new interface library that I want to be the front end for future processing. It should support
# a minimal interface of pushing and image with a pose guess into the server and integrating.
add_library(${PROJECT_NAME}_frontend SHARED
    src/yak_server.cpp
    src/kfusion/tsdf_container.cpp)
if(${PCL_VERSION} VERSION_LESS 1.9)
  target_add_options_and_definitions(${PROJECT_NAME}_frontend ${PCL_DEFINITIONS} ${PCL_COMPILE_OPTIONS})
else()
  target_compile_definitions(${PROJECT_NAME}_frontend PUBLIC ${PCL_DEFINITIONS})
  target_compile_options(${PROJECT_NAME}_frontend PUBLIC ${PCL_COMPILE_OPTIONS})
endif()
target_include_directories(${PROJECT_NAME}_frontend PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_frontend SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_frontend PUBLIC
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
  Eigen3::Eigen)
list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_frontend)

# Marching Cubes Meshing Impl
add_library(${PROJECT_NAME}_marching_cubes SHARED
  src/mc/marching_cubes.cpp
  src/mc/marching_cubes_tables.cpp)
if(${PCL_VERSION} VERSION_LESS 1.9)
  target_add_options_and_definitions(${PROJECT_NAME}_frontend ${PCL_DEFINITIONS} ${PCL_COMPILE_OPTIONS})
else()
  target_compile_definitions(${PROJECT_NAME}_frontend PUBLIC ${PCL_DEFINITIONS})
  target_compile_options(${PROJECT_NAME}_frontend PUBLIC ${PCL_COMPILE_OPTIONS})
endif()
target_include_directories(${PROJECT_NAME}_marching_cubes PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_marching_cubes PUBLIC
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_marching_cubes PUBLIC
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
  Eigen3::Eigen)
list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_marching_cubes)

# Jmeyer Marching Cubes
add_executable(marching_cubes_tests src/mc/marching_cubes_tests.cpp)
if(${PCL_VERSION} VERSION_LESS 1.9)
  target_add_options_and_definitions(${PROJECT_NAME}_frontend ${PCL_DEFINITIONS} ${PCL_COMPILE_OPTIONS})
else()
  target_compile_definitions(${PROJECT_NAME}_frontend PUBLIC ${PCL_DEFINITIONS})
  target_compile_options(${PROJECT_NAME}_frontend PUBLIC ${PCL_COMPILE_OPTIONS})
endif()
target_include_directories(marching_cubes_tests PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(marching_cubes_tests SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS})
target_link_libraries(marching_cubes_tests
  ${PROJECT_NAME}_marching_cubes
  ${PCL_LIBRARIES}
  Eigen3::Eigen)

# Install project files
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN "*.svn" EXCLUDE)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

install(TARGETS marching_cubes_tests
        RUNTIME DESTINATION lib)

# Install CMake targets and config files
install(TARGETS ${PACKAGE_LIBRARIES}
  EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(EXPORT ${PROJECT_NAME}-targets
  NAMESPACE yak:: DESTINATION lib/cmake/${PROJECT_NAME})

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)

# uninstall target
if (NOT TARGET uninstall)
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
                   "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
                   IMMEDIATE @ONLY)

    add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P
                      ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif ()
