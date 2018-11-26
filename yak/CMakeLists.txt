cmake_minimum_required(VERSION 2.8.3)
project(yak)
add_definitions(-std=c++11 -fopenmp)

#compute flags macros
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

find_package(OpenCV REQUIRED COMPONENTS core highgui)
message(STATUS "Found OpenCV At ${OpenCV_INCLUDE_DIRS}")

#find_library(PCL 1.8 REQUIRED COMPONENTS common gpu)

find_package(CUDA REQUIRED)
set(CUDA_ARCH_BIN "3.0 5.0 5.2 6.0 6.1" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
APPEND_TARGET_ARCH_FLAGS()

message(WARNING ${CUDA_NVCC_FLAGS})

find_package(catkin REQUIRED COMPONENTS
  message_generation
  cv_bridge
  image_transport
  rosconsole
  roscpp
  sensor_msgs
  tf
  pcl_ros
  visualization_msgs
  tf_conversions
  interactive_markers

)

add_message_files(
 FILES
 TSDF.msg
 SparseTSDF.msg
)

add_service_files(
 FILES
 GetTSDF.srv
 GetSparseTSDF.srv
 ResetVolume.srv
)


generate_messages(
 DEPENDENCIES
 geometry_msgs
 visualization_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    yak
    ${PROJECT_NAME}_frontend
    ${PROJECT_NAME}_marching_cubes
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    rosconsole
    roscpp
    sensor_msgs
    tf
    pcl_ros
    visualization_msgs
    tf_conversions
    interactive_markers
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

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

add_dependencies(yak ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(yak
  ${CUDA_LIBRARIES}
  ${CUDA_CUDA_LIBRARY}
  ${OpenCV_LIBS}
)

# Marching Cubes Meshing Impl
add_library(${PROJECT_NAME}_marching_cubes
  src/mc/marching_cubes.cpp
  src/mc/marching_cubes_tables.cpp)

add_dependencies(${PROJECT_NAME}_marching_cubes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_marching_cubes
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_frontend
)

# Jmeyer - Create a new interface library that I want to be the front end for future processing. It should support
# a minimal interface of pushing and image with a pose guess into the server and integrating.
add_library(${PROJECT_NAME}_frontend
  src/offline/offline_fusion_server.cpp
  src/offline/tsdf_container.cpp)

add_dependencies(${PROJECT_NAME}_frontend ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_frontend
 yak
 ${catkin_LIBRARIES}
)

# ROS Node Interface for Depth Image Processing
add_executable(kinfu_node
  src/ros/kinfu_node.cpp
  src/ros/ros_rgbd_camera.cpp
  src/ros/kinfu_server.cpp)

add_dependencies(kinfu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(kinfu_node
 yak
 ${catkin_LIBRARIES}
)

# Jmeyer Debug
add_executable(offline_fusion_node src/ros/offline_fusion.cpp)

add_dependencies(offline_fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(offline_fusion_node
 yak
 ${PROJECT_NAME}_marching_cubes
 ${catkin_LIBRARIES}
)

# Jmeyer Marching Cubes
add_executable(marching_cubes_tests src/mc/marching_cubes_tests.cpp)

add_dependencies(marching_cubes_tests ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(marching_cubes_tests
 ${PROJECT_NAME}_marching_cubes
 ${catkin_LIBRARIES}
)

# TODO: What is this?
add_executable(volume_tf_broadcaster src/ros/volume_tf_broadcaster.cpp)

add_dependencies(volume_tf_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(volume_tf_broadcaster
 yak
 ${catkin_LIBRARIES}
)

# TODO: What is this?
add_executable(volume_marker src/ros/volume_marker.cpp)

add_dependencies(volume_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(volume_marker
  yak
  ${catkin_LIBRARIES}
)

# TODO: Why is this here?
#add_definitions(${PCL_DEFINITIONS})

install(TARGETS
  yak
  kinfu_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
