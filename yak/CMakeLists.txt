cmake_minimum_required(VERSION 3.5.0)
project(yak)

# Default to C++14 for cross-compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

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

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/")
message(STATUS "CMAKE MODULE PATH IS ${CMAKE_MODULE_PATH}")

# TODO: Use built-in CUDA language support from CMake 3.8+ (requires different flags)
#enable_language(CUDA)

find_package(OpenMP REQUIRED)

if(NOT TARGET OpenMP::OpenMP_CXX)
    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
endif()

find_package(CUDA REQUIRED)

if (CUDA_VERSION_MAJOR GREATER 8)
    set(CUDA_ARCH_BIN "3.0 5.0 5.2 6.0 6.1" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
else()
    set(CUDA_ARCH_BIN "2.0 2.1 3.0 5.0 5.2 6.0 6.1" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
endif()
APPEND_TARGET_ARCH_FLAGS()

message(STATUS ${CUDA_NVCC_FLAGS})

find_package(OpenCV 3 REQUIRED COMPONENTS core highgui)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io geometry)
set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(${CMAKE_VERSION} VERSION_LESS "3.9.0")
  message(WARNING "You are using CMake version ${CMAKE_VERSION}. Consider upgrading!")
  include_directories(include)  # TODO: Avoid using include_directories at all!
endif()

if ($ENV{ROS_VERSION} VERSION_EQUAL "1")
    # Build for ROS 1 using Catkin
     find_package(catkin)

     catkin_package(
       INCLUDE_DIRS
         include
         ${ROS_INCLUDE_DIRS}
         ${OpenCV_INCLUDE_DIRS}
         ${PCL_INCLUDE_DIRS}
         ${CUDA_INCLUDE_DIRS}
         ${Eigen_INCLUDE_DIRS}
       LIBRARIES
        ${PROJECT_NAME}
         ${PROJECT_NAME}_frontend
         ${PROJECT_NAME}_marching_cubes
         ${CUDA_LIBRARIES}
         ${CUDA_CUDA_LIBRARY}
         ${OpenCV_LIBS}
         OpenMP::OpenMP_CXX
       CATKIN_DEPENDS
       DEPENDS
         OpenCV
         EIGEN3
         CUDA
         PCL
   )
else()
    # Build for ROS 2 using Ament
    find_package(ament_cmake REQUIRED)

    ament_export_include_directories(include
                                    ${OpenCV_INCLUDE_DIRS}
                                    ${PCL_INCLUDE_DIRS}
                                    ${CUDA_INCLUDE_DIRS}
                                    ${Eigen_INCLUDE_DIRS}
                                    )

    ament_export_libraries(${PROJECT_NAME}
                           ${PROJECT_NAME}_frontend
                           ${PROJECT_NAME}_marching_cubes
                           ${CUDA_LIBRARIES}
                           ${CUDA_CUDA_LIBRARY}
                           ${OpenCV_LIBS}
                           ${OpenMP_CXX_LIBRARIES}
                           )

    ament_export_dependencies(OpenCV Eigen3 CUDA OpenMP PCL)

    ament_package()
endif()

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
  src/cuda/tsdf_volume.cu
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${CUDA_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${CUDA_LIBRARIES}
  ${CUDA_CUDA_LIBRARY}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Jmeyer - Create a new interface library that I want to be the front end for future processing. It should support
# a minimal interface of pushing and image with a pose guess into the server and integrating.
add_library(${PROJECT_NAME}_frontend
    src/yak_server.cpp
    src/kfusion/tsdf_container.cpp
)

target_include_directories(${PROJECT_NAME}_frontend PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_frontend
 ${PROJECT_NAME}
 ${PCL_LIBRARIES}
)

# Marching Cubes Meshing Impl
add_library(${PROJECT_NAME}_marching_cubes
  src/mc/marching_cubes.cpp
  src/mc/marching_cubes_tables.cpp)

target_include_directories(${PROJECT_NAME}_marching_cubes PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_marching_cubes
    PUBLIC
  ${PROJECT_NAME}
  OpenMP::OpenMP_CXX
)

# Jmeyer Marching Cubes
add_executable(marching_cubes_tests src/mc/marching_cubes_tests.cpp)

target_include_directories(marching_cubes_tests PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(marching_cubes_tests
    ${PROJECT_NAME}_marching_cubes
    ${PCL_LIBRARIES}
)


if ($ENV{ROS_VERSION} VERSION_EQUAL "1")
    set(ROS_LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    set(ROS_BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else()
    set(ROS_LIB_DESTINATION lib)
    set(ROS_BIN_DESTINATION bin)
endif()

set(ROS_INCLUDE_DESTINATION include)


install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${ROS_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_frontend ${PROJECT_NAME}_marching_cubes
  ARCHIVE DESTINATION ${ROS_LIB_DESTINATION}
  LIBRARY DESTINATION ${ROS_LIB_DESTINATION}
  RUNTIME DESTINATION ${ROS_BIN_DESTINATION}
  )
