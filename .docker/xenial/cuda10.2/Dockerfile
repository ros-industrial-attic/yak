FROM nvidia/cuda:10.2-cudnn7-devel-ubuntu16.04

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    apt-transport-https \
    ca-certificates \
    dirmngr \
    git \
    gnupg \
    gnupg2 \
    libboost-all-dev \
    libeigen3-dev \
    libflann-dev \
    libpcap-dev \
    libproj-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    libvtk6-qt-dev \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO kinetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-core \
    && rm -rf /var/lib/apt/lists/*

# install a newer version of CMake from Kitware's third-party apt repository
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
RUN apt-add-repository 'deb https://apt.kitware.com/ubuntu/ xenial main' \
    && apt-get update \
    && apt-get install -q -y --no-install-recommends cmake \
    && rm -rf /var/lib/apt/lists/*

# build and install PCL 1.8.1 from source
WORKDIR /tmp
RUN \
    git clone --branch pcl-1.8.1 --depth 1 https://github.com/PointCloudLibrary/pcl.git pcl-trunk && \
    cd pcl-trunk && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j 4 && make install && \
    make clean && cd /tmp && rm -rf pcl-trunk

RUN ldconfig

# workaround for yak looking for libcuda.so.1 instead of libcuda.so
RUN ln -s /usr/local/cuda/lib64/stubs/libcuda.so /usr/local/cuda/lib64/stubs/libcuda.so.1

# setup entrypoint
COPY ./docker_entrypoint.sh /
ENTRYPOINT ["/docker_entrypoint.sh"]
CMD ["bash"]
