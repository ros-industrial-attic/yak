cmake_minimum_required(VERSION 2.8.3)

project(yak)
add_definitions(-std=c++11)

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
            set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode arch=compute_${CMAKE_MATCH_2},code=sm_${CMAKE_MATCH_1})
        else()
            # User didn't explicitly specify PTX for the concrete BIN, we assume PTX=BIN
            set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode arch=compute_${ARCH},code=sm_${ARCH})
        endif()
    endforeach()

    # Tell NVCC to add PTX intermediate code for the specified architectures
    string(REGEX MATCHALL "[0-9]+" ARCH_LIST "${ARCH_PTX_WITHOUT_DOTS}")
    foreach(ARCH IN LISTS ARCH_LIST)
        set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode arch=compute_${ARCH},code=compute_${ARCH})
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
#include(cmake/Utils.cmake)
#include(cmake/Targets.cmake)

find_package(OpenCV REQUIRED COMPONENTS core highgui)
message(STATUS "Found OpenCV At ${OpenCV_INCLUDE_DIRS}")

#find_library(OPENVDB_LIBRARY NAMES openvdb)
#find_library(TBB_LIBRARY NAMES tbb)
#find_library(HALF_LIBRARY NAMES Half)
find_library(PCL 1.8 REQUIRED COMPONENTS common gpu)

find_package(CUDA REQUIRED)
if(${CUDA_VERSION_STRING} VERSION_GREATER "4.1")
        set(CUDA_ARCH_BIN "2.0 2.1(2.0) 3.0 5.0 5.2" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
    else()
        set(CUDA_ARCH_BIN "2.0 2.1(2.0)" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
    endif()
APPEND_TARGET_ARCH_FLAGS()
message(STATUS "CUDA_BIN_ARCH ${CUDA_ARCH_BIN}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rosconsole
  roscpp
  sensor_msgs
  tf
  pcl_ros
  visualization_msgs
  tf_conversions
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
 #GetMesh.srv
)


generate_messages(
 DEPENDENCIES
 geometry_msgs
 visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yak
  CATKIN_DEPENDS cv_bridge image_transport rosconsole roscpp sensor_msgs pcl_ros visualization_msgs Eigen3 tf_conversions
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
message("debugging")
message("PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message("CUDA_INCLUDE_DIRS: ${CUDA_INCLUDE_DIRS}")
message("CUDA_CUDA_LIBRARY: ${CUDA_CUDA_LIBRARY}")

set(HAVE_CUDA 1)
#list(APPEND CUDA_NVCC_FLAGS "-arch;compute_20")
list(APPEND CUDA_NVCC_FLAGS "-gencode;arch=compute_20,code=sm_20;-gencode;arch=compute_20,code=sm_21;-gencode;arch=compute_30,code=sm_30;-gencode;arch=compute_35,code=sm_35")
  list(APPEND CUDA_NVCC_FLAGS "-Xcompiler;-fPIC;")

include_directories(${PROJECT_SOURCE_DIR}/include/yak)
include_directories(${PROJECT_SOURCE_DIR}/src)

link_directories(${PCL_LIBRARY_DIRS})

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
      src/ros/ros_rgbd_camera.cpp
      src/ros/kinfu_server.cpp
)

 target_link_libraries(yak
   ${catkin_LIBRARIES}
   ${CUDA_LIBRARIES}
   ${CUDA_CUDA_LIBRARY}
   ${OpenCV_LIBS}
   #openvdb
   #Half
   #tbb
 )

add_executable(kinfu_node
  src/kinfu_node.cpp)

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(kinfu_node
 yak
 ${catkin_LIBRARIES}
 ${CUDA_LIBRARIES}
 ${CUDA_CUDA_LIBRARY}
 ${OpenCV_LIBS}
)

 install(TARGETS yak kinfu_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 )
