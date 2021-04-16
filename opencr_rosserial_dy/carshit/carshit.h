#include <Servo.h>

#define MAX_RPM 390
#define MIN_RPM 80
#define DELAY_TIMER 50
#define MOTOR_WRITE 3
#define DIR_WRITE 52



void steering (int turn_angle, Servo drive);
void driving (int rpm, int time_accel, int dir);
float speed_control();
int rpm_to_pwm(int rpm_x);
int pwm_to_rpm(int pwm_x);
void PID(int goal_rpm);
void encoder_counter();
//void handler_nh(void);
