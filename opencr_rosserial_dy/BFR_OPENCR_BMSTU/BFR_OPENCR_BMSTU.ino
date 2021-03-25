
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include  "robot.h" // подключаем файл с нужными функциями
#define BAUDRATE 1000000
#define WHEEL_NUM  4
#define CMD_VEL_TIMEOUT 2000 //2 seconds
#define Pi 3.14159265359
sensor_msgs::JointState joint_state;
      
//ros::NodeHandle nh;//объект ноудхэндл (по сути создаем ноду)
typedef ros::NodeHandle_<ArduinoHardware, 11, 11, 16384, 16384> NodeHandle;
NodeHandle nh;  
static float target_pose[2] = {3, 4};
static float target_vel[2] = {0, 0};
static uint8_t target_acc[2] = {20, 20};
static float current_velocity[2] =  {0.0, 0.0};
static float current_effort[2] = {0.0, 0.0};
static float current_pose[2] = {0.0, 0.0};
void pose_cb(const geometry_msgs::Vector3& pose){//колбэк топика положения подвеса
  target_pose[0] = pose.x;
  target_pose[1] = pose.y;
}
void vel_cb(const geometry_msgs::Vector3& vel){//колбэк топика скорости подвеса
  target_vel[0] = vel.x;
  target_vel[1] = vel.y;
}
void acc_cb(const geometry_msgs::Vector3& acc){//колбэк топика скорости подвеса
  target_acc[0] = uint8_t(acc.x);
  target_acc[1] = uint8_t(acc.y);
}
int time_millis[7] = {0,0,0,0,0,0,0};
int light = 0;
ros::Publisher joint_state_pub("camera_motors_position", &joint_state);//создаем объект паблишер, чтобы публиковать информацию о своем состоянии (свободен/занят)
ros::Subscriber<geometry_msgs::Vector3> sub_pose("target_angles", pose_cb);
ros::Subscriber<geometry_msgs::Vector3> sub_vel("target_velocity", vel_cb);
ros::Subscriber<geometry_msgs::Vector3> sub_acc("target_acceleration", acc_cb);
HardwareTimer Timer(TIMER_CH4);//таймер для обновления информации приходящей из ROS-а
void setup() {
  define_joint_state();
  init();//функция инициализирования Dxl устройств
  nh.initNode(); //инициализируем ноду
  nh.getHardware()->setBaud(BAUDRATE);//задаем частоту общения с росом 
  nh.advertise(joint_state_pub);//инициализируем паблишеры и субскрайберы
  nh.subscribe(sub_pose);
  nh.subscribe(sub_acc);
  nh.subscribe(sub_vel);
  Timer.stop();//иниц/иализируем таймеры         
  Timer.setPeriod(5000);// in microseconds
  Timer.attachInterrupt(handler_nh);//при прерывании вызывается функция handler_nh
  Timer.start();
  
}

void loop(){
  if(millis() - time_millis[0] > 50)
  {
    int dt = millis() - time_millis[0];
    time_millis[0] = millis();
    set_acc(target_acc[0],target_acc[1]);
    set_speed(target_vel[0],target_vel[1]);
    move_motors(target_pose[0], target_pose[1]);

    
    read_joint_state(current_pose, current_velocity, current_effort);
    joint_state.header.stamp = nh.now();
    joint_state.velocity = current_velocity;
    joint_state.effort = current_effort;
    joint_state.position = current_pose;
    joint_state_pub.publish(&joint_state); 
  }
}
void define_joint_state(){
  static char *joint_state_name[] = {(char*)"bottom_motor", (char*)"top_motor"};
  joint_state.header.frame_id = "base_link";
  joint_state.name            = joint_state_name;
  joint_state.name_length     = 2;
  joint_state.position_length = 2;
  joint_state.velocity_length = 2;
  joint_state.effort_length   = 2;
}

void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}
void handler_nh(void){//связываемся с ROS-ом по прерыванию
  nh.spinOnce();// метод spinOnce осуществляет один цикл общения с ROS-ом, если его не вызывать с заданной частотой, то устройства перестанут быть связаны
}
