std_msgs/Header  header
#goal velocities for wheels  
int16 cmd_vel_bl  
int16 cmd_vel_br 
int16 cmd_vel_fl 
int16 cmd_vel_fr
#measured velocities for wheels
int16 vel_bl  
int16 vel_br 
int16 vel_fl  
int16 vel_fr  
#measured positions for wheels
int16 pos_bl  
int16 pos_br  
int16 pos_fl  
int16 pos_fr  
#measured currents for wheels
int16 curr_bl  
int16 curr_br  
int16 curr_fl  
int16 curr_fr  
#measured accelerations for wheels
int16 acc_bl  
int16 acc_br  
int16 acc_fl  
int16 acc_fr  
#goal positions for neck and grabbers
int16 pos_n  
int16 pos_gl  
int16 pos_gr  
#sensory data
float32 battery_voltage  
 #IMU data
float32 imu_angular_vel_x  
float32 imu_angular_vel_y  
float32 imu_angular_vel_z  
float32 imu_linear_acc_x  
float32 imu_linear_acc_y  
float32 imu_linear_acc_z  
float32 imu_magnetic_x 
float32 imu_magnetic_y
float32 imu_magnetic_z
float32 imu_orientation_w 
float32 imu_orientation_x 
float32 imu_orientation_y 
float32 imu_orientation_z 