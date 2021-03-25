#include <ros.h>
#include "carshit.h"
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#define BAUDRATE 1000000
//HardwareTimer Timer(TIMER_CH1);
HardwareTimer Timer(TIMER_CH4);
Servo wheel;

ros::NodeHandle nh;
float speed_data;
sensor_msgs::JointState joint_state;

static float target_pose = 0;
static float target_vel = 0;
float target_time= 0;
float target_time_pos = 0;
float target_time_vel = 0;
float dir_move = 0;
static float current_pose[2] = {0.0, 0.0};
static float target_posse[2] = {0, 0};
int time_millis[7] = {0,0,0,0,0,0,0};
void pose_cb(const geometry_msgs::Vector3& pose){//колбэк топика положения подвеса
  target_pose = pose.x;
  //target_time_pos = pose.y;
  
}
void vel_cb(const geometry_msgs::Vector3& vel){//колбэк топика скорости подвеса
  target_vel = vel.x;
  target_time_vel = vel.y;
  dir_move = vel.z;
  
}

ros::Publisher joint_state_pub("camera_motors_position", &joint_state);
ros::Subscriber<geometry_msgs::Vector3> sub_pose("target_angles", pose_cb);
ros::Subscriber<geometry_msgs::Vector3> sub_vel("target_velocity", vel_cb);

void setup() {
  nh.initNode(); //инициализируем ноду
  nh.getHardware()->setBaud(BAUDRATE);//задаем частоту общения с росом 
  nh.advertise(joint_state_pub);//инициализируем паблишеры и субскрайберы
  nh.subscribe(sub_pose);
  nh.subscribe(sub_vel);
  
  wheel.attach(9);
  wheel.write(90);
  pinMode(DIR_WRITE, OUTPUT);
  digitalWrite(DIR_WRITE, HIGH);
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  pinMode(MOTOR_WRITE, OUTPUT);
  analogWrite(MOTOR_WRITE, 0);
  pinMode(4, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(64, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(4), encoder_counter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(7), encoder_counter, CHANGE);
  //Timer.stop();
  //Timer.setPeriod(DELAY_TIMER*1000);
  //Timer.attachInterrupt(speed_control);
 // Timer.start();
  Timer.stop();//иниц/иализируем таймеры         
  Timer.setPeriod(5000);// in microseconds
  Timer.attachInterrupt(handler_nh);//при прерывании вызывается функция handler_nh
  Timer.start();

  
  //Serial.begin(9600); 
}

void loop() {

  if(millis() - time_millis[0] > 50)
  {
   int dt = millis() - time_millis[0];
   steering(target_pose, wheel);
   driving(target_vel, target_time_vel, dir_move);
   
   speed_data = speed_control();
   joint_state.header.stamp = nh.now();
   target_posse[0] = speed_data;
   joint_state.velocity = target_posse;
   joint_state.effort = 0;
   joint_state.position = current_pose;
   joint_state_pub.publish(&joint_state); 
   
//  delay(1000);
//  steering(100, -20, wheel);
//  delay(1000);
//  steering(100, -60, wheel);
//  delay(1000);
//  steering(100, 30, wheel);
//  delay(1000);
//  driving(300, 3, 1, 5);
//  driving(250, 5, -1);
  
}
}

void handler_nh(void){//связываемся с ROS-ом по прерыванию
  nh.spinOnce();// метод spinOnce осуществляет один цикл общения с ROS-ом, если его не вызывать с заданной частотой, то устройства перестанут быть связаны
}
