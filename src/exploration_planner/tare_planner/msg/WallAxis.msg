# Dominant building-wall orientation estimated from the room_segmentation wall
# planes (plane_infos_). A single Manhattan grid angle; the two building axes are
# {yaw_rad, yaw_rad + pi/2}. Consumed by QuadrantManager as the primary axis source.
std_msgs/Header header
float64 yaw_rad         # dominant grid angle in [0, pi/2), map frame
float64 confidence      # fraction of wall length aligned to the grid, [0, 1]
float64 support_length  # aligned (inlier) wall length, meters
bool valid              # false when there are no / too few walls to estimate
