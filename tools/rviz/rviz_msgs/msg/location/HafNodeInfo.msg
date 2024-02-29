std_msgs/Header header

rviz_msgs/Header proto_header

int32 NodeType_INS = 0
int32 NodeType_WO = 1
int32 NodeType_VIO = 2
int32 NodeType_MapMatcher = 3
int32 NodeType_DR = 4
int32 NodeType_PoseEstimate = 5
# NodeType
int32 type

geometry_msgs/Point pos_wgs
geometry_msgs/Point attitude
geometry_msgs/Quaternion quaternion
geometry_msgs/Vector3 linear_velocity
uint32 sys_status
uint32 gps_status
float32 heading
geometry_msgs/Point pos_gcj02
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 linear_acceleration
