std_msgs/Header header
std_msgs/Int64 fsm_state

float64 mpc_solve_time

float64 yaw
float64 yaw_dot

float64 des_vel_norm
float64 des_acc_norm

geometry_msgs/Point desire_pos
geometry_msgs/Vector3 desire_vel
geometry_msgs/Vector3 desire_att
geometry_msgs/Vector3 desire_omg

geometry_msgs/Point feedback_pos
geometry_msgs/Vector3 feedback_vel
geometry_msgs/Vector3 feedback_att
geometry_msgs/Vector3 feedback_omg

geometry_msgs/Point pos_error
geometry_msgs/Vector3 vel_error
geometry_msgs/Vector3 att_error
geometry_msgs/Vector3 omg_error