# Yak

[![GitHub Actions status](https://github.com/ros-industrial/yak/workflows/CI/badge.svg?branch=devel)](https://github.com/ros-industrial/yak/actions)
[![GitHub issues open](https://img.shields.io/github/issues/ros-industrial/yak.svg?)](https://github.com/ros-industrial/yak/issues)
[![license - MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)


Yak is a library for integrating depth images into Truncated Signed Distance Fields (TSDFs). It is currently supported for Ubuntu 16.04 and Ubuntu 18.04. An example ROS node using Yak is provided in the [yak_ros](https://github.com/ros-industrial/yak_ros) repository.

## Technical Background

A TSDF is a probabilistic representation of a solid surface in 3D space. It's a useful tool for combining many noisy incomplete sensor readings into a single smooth and complete model.

To break down the name:

Distance Field: Each voxel in the volume contains a value that represents its metric distance from the closest point on the surface. Voxels very far from the surface have high-magnitude distance values, while those near the surface have values approaching zero.

Signed: Voxels outside the surface have positive distances, while voxels inside the surface have negative distances. This allows the representation of solid objects. The distance field represents a gradient that shifts from positive to negative as it crosses the surface.

Truncated: Voxels further than a specified distance from the isosurface have their values capped at +/- 1 because we are only interested in precisely representing the region of the volume close to solid objects.

The TSDF algorithm can be efficiently parallelized on a general-purpose graphics processor, which allows data from RGB-D cameras to be integrated into the volume in real time. Numerous observations of an object from different perspectives average out noise and errors due to specular highlights and interreflections, producing a smooth continuous surface. This is a key advantage over equivalent point-cloud-centric strategies, which require additional processing to distinguish between engineered features and erroneous artifacts in the scan data. The volume can be converted to a triangular mesh using the Marching Cubes algorithm for consumption by application-specific processes.

Yak is intended as a flexible library that gives the end user (you!) a lot of freedom over when data gets integrated, which poses are used, whether Iterative Closest Point plays a role, etc.

![An aluminum part reconstructed as a TSDF and meshed with the Marching Cubes algorithm.](/aluminum_channel_mesh.png)

## Dependencies

### CMake

Yak requires CMake 3.10.0 or newer in order to take advantage of improved support for CUDA. If you're using Ubuntu 18.04 or newer you should already have a suitable version. If you're using an older distribution (e.g. 16.04) you will need to install a compatible version of CMake yourself.

1. Install pip (if it isn't already installed):
```
sudo apt install python-pip
```

2. Use pip to install CMake locally, which will not overwrite the system installation of CMake. As of September 5 2019 this installs CMake 3.14.4 on Ubuntu 16.04.
```
pip install --user cmake --upgrade
```

3. (Optional, depending on the specifics of your environment) Prepend the directory where CMake was just installed to the PATH environment variable. You may also add this to `.bashrc`.
```
export PATH=:/home/YOUR_USERNAME/.local/bin:$PATH
```

### CUDA

Yak requires an NVidia GPU and CUDA 9.0 or newer.

1. CUDA depends on Nvidia third-party drivers and will not work with the default Nouveau drivers. Determine which version of the Nvidia driver is compatible with your computer's GPU by running `ubuntu-drivers devices`. Usually the driver version listed as "third-party free recommended" is suitable. You can install the recommended driver using `sudo ubuntu-drivers autoinstall`. Example output listed below:

```
== /sys/devices/pci0000:00/0000:00:02.0/0000:02:00.0 ==
modalias : pci:v000010DEd000017C8sv00001458sd000036CBbc03sc00i00
vendor   : NVIDIA Corporation
model    : GM200 [GeForce GTX 980 Ti]
driver   : nvidia-driver-418 - third-party free recommended
driver   : nvidia-driver-410 - third-party free
driver   : nvidia-driver-390 - distro non-free
driver   : xserver-xorg-video-nouveau - distro free builtin
```

2. Consult the chart below (copied from [here](https://docs.nvidia.com/deploy/cuda-compatibility/index.html#binary-compatibility__table-toolkit-driver) as of September 5 2019) to find which CUDA version is compatible with the drivers supported by your GPU. Yak is currently not compatible with CUDA <= 8.0.

| CUDA Toolkit | Linux x86_64 Driver Version |
|---|---|
| CUDA 10.1 (10.1.105) | >= 418.39 |
| CUDA 10.0 (10.0.130) | >= 410.48 | 
| CUDA 9.2 (9.2.88) | >= 396.26 |
| CUDA 9.1 (9.1.85) | >= 390.46 |
| CUDA 9.0 (9.0.76) | >= 384.81 |
| CUDA 8.0 (8.0.61 GA2) | >= 375.26 |
| CUDA 8.0 (8.0.44) | >= 367.48 | 
| CUDA 7.5 (7.5.16) | >= 352.31 |
| CUDA 7.0 (7.0.28) | >= 346.46 |

3. Specific installation instructions vary depending on the combination of Ubuntu version, CUDA version, and Nvidia driver version. Please read the linked instructions carefully: they are very important, and CUDA may not work correctly if you skip a step!

- Package manager installation is usually the easiest method, especially if you have Ubuntu 18.04 and a new GPU. [Download a compatible version of CUDA from here](https://developer.nvidia.com/cuda-toolkit-archive) (pick "Installer Type: deb (network)") and then [follow these instructions](https://docs.nvidia.com/cuda/cuda-quick-start-guide/index.html#ubuntu-x86_64-deb). 

- Nvidia doesn't release debians for every combination of Ubuntu release and CUDA version. If the package manager installation method fails to find a compatible .deb (e.g. you have Ubuntu 18.04 but your GPU only supports driver version 384), [download the runfile version of the CUDA installer from here](https://developer.nvidia.com/cuda-toolkit-archive) (pick "Installer Type: runfile (local)") and then [follow the runfile installation instructions](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#runfile). Since you have already installed the NVidia driver in step 1 you do not need to reboot your system into non-GUI mode, and when the runfile dialog asks if you would like to install these drivers you may decline.

4. Follow the instructions in [Post-installation Actions](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#post-installation-actions). It is not necessary to complete the steps listed under "POWER9 Setup" -- you probably do not have a POWER9-compatible system unless you work with supercomputers.

### OpenMP

Yak requires OpenMP for parallelization of the marching cubes surface reconstruction process.
```
sudo apt install libomp-dev
```

### OpenCV

Yak requires OpenCV 3.0 or newer. If you are using ROS then this dependency is already satisfied. Otherwise you will need to install it yourself:
```
sudo apt install libopencv-dev
```

### Eigen

Yak requires Eigen 3. If you are using ROS then this dependency is already satisfied. Otherwise you will need to install it yourself:
```
sudo apt install libeigen3-dev
```

### PCL

Yak requires PCL 1.8 or newer. If you are using ROS then this dependency is already satisfied. Otherwise you will need to install it yourself:
```
sudo apt install libpcl-dev
```

## Installation

### Build with ROS (recommended)

Installing ROS is beyond the scope of this readme. There are installation instructions [here for ROS1](http://wiki.ros.org/melodic/Installation/Ubuntu) and [here for ROS2](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/).

Clone this package into the `src` directory of a ROS workspace. For ROS1 use `catkin build` to build the workspace. For ROS2 use `colcon build` to build the workspace.

Take a look at the [yak_ros](https://github.com/ros-industrial/yak_ros) package for example implementations of ROS nodes using Yak.

### Build Standalone (in development)

```
git clone https://github.com/ros-industrial/yak.git
cd yak
mkdir build
cd build
cmake ../yak
make
sudo make install
```

## Readme TODOs:

- Minimally-functional example
- Documentation of services, inputs, outputs, etc.

