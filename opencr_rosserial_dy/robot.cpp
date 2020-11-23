#include "robot.h"
#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" 
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   
#define BAUDRATE  1000000  // скорость опроса датчиков
#define DXL_0    1  // передний левый
#define DXL_1    2  // передний правый
uint8_t dxl_id[2] = {DXL_0, DXL_1};
uint8_t d_gain = 0; //128
uint8_t i_gain = 16; //16
uint8_t p_gain = 16; //16

DynamixelWorkbench dxl_wb;

void init(){// функция инициализации dxl устройств  
  const char *log;  
  bool result = false ;
  uint16_t model_number = 0;
  //Serial.begin(9600);
  //while(!Serial);
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  for(int i = 0; i<2; i++){
    result = dxl_wb.ping(dxl_id[i], &model_number, &log); 
    //Serial.println(log);
  }
  /*
  for(int i = 0; i<1; i++){
    dxl_wb.jointMode(dxl_id[i], (int32_t)10, (int32_t)10, &log);  
    //Serial.println(log);
    dxl_wb.writeRegister(dxl_id[i], uint16_t(26), uint16_t(1), &d_gain, &log);
//    Serial.println(log);
    dxl_wb.writeRegister(dxl_id[i], uint16_t(27), uint16_t(1), &i_gain, &log);
  //  Serial.println(log);
    dxl_wb.writeRegister(dxl_id[i], uint16_t(28), uint16_t(1), &p_gain, &log);
//    Serial.println(log);
    
  }
  */
  dxl_wb.wheelMode(dxl_id[0], 0, &log);
  dxl_wb.jointMode(dxl_id[1], (int32_t)10, (int32_t)10, &log);  
    //Serial.println(log);
    dxl_wb.writeRegister(dxl_id[1], uint16_t(26), uint16_t(1), &d_gain, &log);
//    Serial.println(log);
    dxl_wb.writeRegister(dxl_id[1], uint16_t(27), uint16_t(1), &i_gain, &log);
  //  Serial.println(log);
    dxl_wb.writeRegister(dxl_id[1], uint16_t(28), uint16_t(1), &p_gain, &log);
//    Serial.println(log);
  
}

void move_motors(float pose_0,float pose_1){
  const char *log;
  bool result = false;
  //int32_t angle_0= int32_t(pose_0*652.2);
  //result = dxl_wb.goalPosition((uint8_t)DXL_0, angle_0, &log);
  //Serial.println(log);
  if(pose_1 >= 3 && pose_1 <= 6){
    int32_t angle_1= int32_t(pose_1*652.2);
    result = dxl_wb.goalPosition((uint8_t)DXL_1, angle_1, &log);
  }
//  Serial.println(log);
}
void set_speed(float speed_0,float speed_1){
  const char *log;
  bool result = false;
  int32_t sp_0 = int32_t(speed_0*1);  
  result = dxl_wb.goalVelocity(DXL_0, sp_0, &log);
//  Serial.println(log);
 // result = dxl_wb.goalVelocity((uint8_t)DXL_1, sp_1,&log);
//  Serial.println(log);
}

void set_acc(uint8_t acc_0,uint8_t acc_1){
  const char *log;
  bool result = false;
  result = dxl_wb.writeRegister((uint8_t)DXL_0, (uint16_t)73, (uint16_t)1, &acc_0,&log);
 // Serial.println(log);
  result = dxl_wb.writeRegister((uint8_t)DXL_1, (uint16_t)73, (uint16_t)1, &acc_1,&log);
  //Serial.println(log);
}
void read_joint_state(float*current_pose ,float* current_velocity, float* current_effort){
  const char* log;
  for(int i = 1; i<2; i++){
    int32_t get_data = 0;
    dxl_wb.getPresentVelocityData(dxl_id[i], &get_data, &log);
    if(get_data > 1023)
      current_velocity[i] = -get_data*0.01152;
    else
      current_velocity[i] = -get_data*0.01152;

    uint32_t get_data_eff = 0;
    dxl_wb.readRegister(dxl_id[i], (uint16_t)40, (uint16_t)2, &get_data_eff, &log);
    if(abs(get_data_eff) > 1023)
      get_data_eff = 0;
    current_effort[i] = get_data_eff*0.001*294.2;
    float get_data_pos = 0;
    dxl_wb.getRadian(dxl_id[i], &get_data_pos, &log);
    current_pose[i] = get_data_pos + 3.14159265359;
  }
}
float check_voltage(){
  int adc_value = analogRead(BDPIN_BAT_PWR_ADC);  
  float vol_value = map(adc_value, 0, 1023, 0, 330*57/10);
  vol_value = vol_value/100;
  return vol_value;
}
