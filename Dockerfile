#docker build -t rosindustrial/yak:kinetic .
FROM rosindustrial/noether-nvidia:kinetic
WORKDIR /
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-kinetic-perception \
	libopenvdb-dev \
	ros-kinetic-interactive-markers \
	libtbb-dev \
	libopenexr-dev \
	ros-kinetic-octomap-ros \
	ros-kinetic-moveit-ros-planning-interface && \
	apt-get clean && \
	rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
