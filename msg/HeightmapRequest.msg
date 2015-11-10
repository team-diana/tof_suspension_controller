# See https://github.com/team-diana/heightmap/blob/master/srv/Query.srv for format

# Rectangular area to be sampled, axis-aligned to the frame of
# reference specified by corner.header.frame_id
geometry_msgs/PointStamped corner
float64  x_size
float64  y_size

# Desired resolution. In the response, x_size and y_size will be equal
# to these two members of the request
uint32  x_samples
uint32  y_samples