# Complete world-axis-aligned local snapshot. Missing occupancy is traversable.
std_msgs/Header header
uint64 epoch
uint64 version
float64 resolution
geometry_msgs/Point origin
uint32[3] size
uint32 inflation_steps
# x + size[0]*(y + size[1]*z); least significant bit first.
# Half-open bounds: origin <= position < origin + resolution*size.
uint8[] occupied_bits
uint8[] inflated_bits
