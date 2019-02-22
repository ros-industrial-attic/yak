cmake_minimum_required(VERSION 3.5)
project(yak)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### START CUDA MACROS
MACRO(CUDA_COMPUTE_TARGET_FLAGS arch_bin arch_ptx cuda_nvcc_target_flags)
    string(REGEX REPLACE "\\." "" ARCH_BIN_WITHOUT_DOTS "${${arch_bin}}")
    string(REGEX REPLACE "\\." "" ARCH_PTX_WITHOUT_DOTS "${${arch_ptx}}")

    set(cuda_computer_target_flags_temp "")

    # Tell NVCC to add binaries for the specified GPUs
    string(REGEX MATCHALL "[0-9()]+" ARCH_LIST "${ARCH_BIN_WITHOUT_DOTS}")
    foreach(ARCH IN LISTS ARCH_LIST)
        if (ARCH MATCHES "([0-9]+)\\(([0-9]+)\\)")
            # User explicitly specified PTX for the concrete BIN
            set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode;arch=compute_${CMAKE_MATCH_2},code=sm_${CMAKE_MATCH_1};)
        else()
            # User didn't explicitly specify PTX for the concrete BIN, we assume PTX=BIN
            set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode;arch=compute_${ARCH},code=sm_${ARCH};)
        endif()
    endforeach()

    # Tell NVCC to add PTX intermediate code for the specified architectures
    string(REGEX MATCHALL "[0-9]+" ARCH_LIST "${ARCH_PTX_WITHOUT_DOTS}")
    foreach(ARCH IN LISTS ARCH_LIST)
        set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode;arch=compute_${ARCH},code=compute_${ARCH};)
    endforeach()

    set(${cuda_nvcc_target_flags} ${cuda_computer_target_flags_temp})
ENDMACRO()

MACRO(APPEND_TARGET_ARCH_FLAGS)
    set(cuda_nvcc_target_flags "")
    CUDA_COMPUTE_TARGET_FLAGS(CUDA_ARCH_BIN CUDA_ARCH_PTX cuda_nvcc_target_flags)
    if (cuda_nvcc_target_flags)
        message(STATUS "CUDA NVCC target flags: ${cuda_nvcc_target_flags}")
        list(APPEND CUDA_NVCC_FLAGS ${cuda_nvcc_target_flags})
    endif()
ENDMACRO()
### END CUDA MACROS

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/")
message(STATUS "CMAKE MODULE PATH IS ${CMAKE_MODULE_PATH}")

find_package(CUDA REQUIRED)
if (CUDA_VERSION_MAJOR GREATER 8)
    set(CUDA_ARCH_BIN "3.0 5.0 5.2 6.0 6.1" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
else()
    set(CUDA_ARCH_BIN "2.0 2.1 3.0 5.0 5.2 6.0 6.1" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
endif()
APPEND_TARGET_ARCH_FLAGS()

message(STATUS ${CUDA_NVCC_FLAGS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui)
find_package(PCL 1.8 REQUIRED COMPONENTS common io geometry)
set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

# Core CUDA Library for depth image processing
cuda_add_library(yak
  src/core.cpp
  src/device_memory.cpp
  src/imgproc.cpp
  src/kinfu.cpp
  src/precomp.cpp
  src/projective_icp.cpp
  src/tsdf_volume.cpp
  src/cuda/imgproc.cu
  src/cuda/proj_icp.cu
  src/cuda/tsdf_volume.cu
)

ament_target_dependencies(yak ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(yak
  ${CUDA_LIBRARIES}
  ${CUDA_CUDA_LIBRARY}
  ${OpenCV_LIBS}
  ${rclcpp_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Marching Cubes Meshing Impl
add_library(${PROJECT_NAME}_marching_cubes
  src/mc/marching_cubes.cpp
  src/mc/marching_cubes_tables.cpp)

ament_target_dependencies(${PROJECT_NAME}_marching_cubes ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_marching_cubes
  ${rclcpp_LIBRARIES}
  ${PROJECT_NAME}_frontend
)

# Jmeyer - Create a new interface library that I want to be the front end for future processing. It should support
# a minimal interface of pushing and image with a pose guess into the server and integrating.
add_library(${PROJECT_NAME}_frontend
  src/ros/yak_server.cpp
  src/offline/tsdf_container.cpp)

target_include_directories(${PROJECT_NAME}_frontend
    PUBLIC include/offline/ include/ros/
)

ament_target_dependencies(${PROJECT_NAME}_frontend ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_frontend
 yak
 ${rclcpp_LIBRARIES}
 ${PCL_LIBRARIES}
)

# Jmeyer Marching Cubes
add_executable(marching_cubes_tests src/mc/marching_cubes_tests.cpp)
ament_target_dependencies(marching_cubes_tests ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(marching_cubes_tests
 ${PROJECT_NAME}_marching_cubes
 ${PCL_LIBRARIES}
 ${rclcpp_LIBRARIES}
)

ament_export_include_directories(${PROJECT_NAME}_INCLUDE_DIRS ${PCL_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_frontend ${PROJECT_NAME}_marching_cubes)
ament_export_dependencies(PCL EIGEN3)

install(TARGETS
  yak
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

ament_package()

