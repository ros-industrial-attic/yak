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

#Binary serialization of distances/weights. 
# The first 16 bits are a half-precision floating point value representing the TSDF. The second 16 bits are
# an unsigned 16 bit weight value.
uint32[] data