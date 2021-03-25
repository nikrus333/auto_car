#include <DynamixelWorkbench.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <IMU.h>
#include <sensor_msgs/Imu.h>
void init();
void move_motors(float pose_0,float pose_1);
void set_speed(float speed_0,float speed_1);
void set_acc(uint8_t acc_0,uint8_t acc_1);
void read_joint_state(float*current_pose ,float* current_velocity, float* current_effort);
