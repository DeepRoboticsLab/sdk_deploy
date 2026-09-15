# Published at 10 Hz. Joint positions and base_rpy are radians;
# joint velocities and base_angular_velocity are rad/s in the SDK/body frames.
std_msgs/Header header
string robot
string command_source
string current_state
uint8 current_state_id
string requested_state
uint8 requested_state_id
uint8 safety_mode
geometry_msgs/Twist command
bool command_timed_out
uint64 rejected_commands
uint64 command_updates
float64 joint_stamp
float64 imu_stamp
bool telemetry_fresh
sensor_msgs/JointState joints
geometry_msgs/Vector3 base_rpy
geometry_msgs/Vector3 base_angular_velocity
