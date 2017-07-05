Header header

#Volume dimension, in meters
float32 size_x
float32 size_y
float32 size_z

#number of voxels in each dimension
int32 num_voxels_x
int32 num_voxels_y
int32 num_voxels_z

#Truncation distance, in meters
float32 truncation_dist

#Maximum tsdf weight
int32 max_weight

#Pose of the TSDF with respect to the camera origin
geometry_msgs/Pose pose

# Binary serialization of distances/weights.
# Use a 3D implementation of compressed row storage to improve the packing efficiency of the data and let bigger TSDFs be serialized from the GPU.
# For each voxel with nonzero weight.
# The first 16 bits are a half-precision floating point value representing the TSDF.
# The second 16 bits are an unsigned 16 bit weight value.
uint32[] data

# Row (X) coordinate of each voxel in data.
uint16[] rows
# Col (Y) coordinate of each contiguous series of row coordinates.
uint16[] cols
# Sheet (Z) coordinate of each contiguous series of col coordinates.
uint16[] sheets
