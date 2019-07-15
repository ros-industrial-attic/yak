# Yak

Yak is a library for integrating depth images into Truncated Signed Distance Fields (TSDFs). An example ROS node using Yak is provided in the [yak_ros](https://github.com/ros-industrial/yak_ros) repository.

A TSDF is a probabilistic representation of a solid surface in 3D space. It's a useful tool for combining many noisy incomplete sensor readings into a single smooth and complete model.

To break down the name:

Distance Field: Each voxel in the volume contains a value that represents its metric distance from the closest point on the surface. Voxels very far from the surface have high-magnitude distance values, while those near the surface have values approaching zero.

Signed: Voxels outside the surface have positive distances, while voxels inside the surface have negative distances. This allows the representation of solid objects. The distance field represents a gradient that shifts from positive to negative as it crosses the surface.

Truncated: Voxels further than a specified distance from the isosurface have their values capped at +/- 1 because we are only interested in precisely representing the region of the volume close to solid objects.

The TSDF algorithm can be efficiently parallelized on a general-purpose graphics processor, which allows data from RGB-D cameras to be integrated into the volume in real time. Numerous observations of an object from different perspectives average out noise and errors due to specular highlights and interreflections, producing a smooth continuous surface. This is a key advantage over equivalent point-cloud-centric strategies, which require additional processing to distinguish between engineered features and erroneous artifacts in the scan data. The volume can be converted to a triangular mesh using the Marching Cubes algorithm for consumption by application-specific processes.

Yak is intended as a flexible library that gives the end user (you!) a lot of freedom over when data gets integrated, which poses are used, whether Iterative Closest Point plays a role, etc.

![An aluminum part reconstructed as a TSDF and meshed with the Marching Cubes algorithm.](/aluminum_channel_mesh.png)

# Readme TODOs:

- Basic setup instructions
- Minimally-functional example
- Documentation of services, inputs, outputs, etc.

