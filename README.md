# yak

yak (Yet Another Kinfu) is a library and ROS wrapper for integrating depth images into Truncated Signed Distance Fields (TSDFs).

A TSDF is a probabilistic representation of a solid surface in 3D space. It's a useful tool for combining many noisy incomplete sensor readings into a single smooth and complete model.

To break down the name:

Distance Field: Each voxel in the volume contains a value that represents its metric distance from the closest point on the surface. Voxels very far from the surface have high-magnitude distance values, while those near the surface have values approaching zero.

Signed: Voxels outside the surface have positive distances, while voxels inside the surface have negative distances. This allows the representation of solid objects. The distance field becomes a gradient that shifts from positive to negative as it crosses the surface.

Truncated: Voxels further than a specified distance from the isosurface have their values capped at +/- 1.

yak is intended as a flexible library that gives the end user (you!) a lot of freedom over when data gets integrated, which poses are used, whether Iterative Closest Point plays a role, etc. This means that some work is required on your end to get everything plugged in and running.

![An aluminum part reconstructed as a TSDF and meshed with the Marching Cubes algorithm.](/aluminum_channel_mesh.png)

# Readme TODOs:

- Basic setup instructions
- Minimally-functional example
- Documentation of services, inputs, outputs, etc.


# (Obsolete?) Build with Docker:
```
nvidia-docker run -v "<absolute path to your yak workspace:/yak_ws>" rosindustrial/yak:kinetic catkin build --workspace /yak_ws -DCMAKE_LIBRARY_PATH=/usr/local/nvidia/lib64/
```
