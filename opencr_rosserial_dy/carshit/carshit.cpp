#include "carshit.h"
#include "Arduino.h"


uint16_t enc = 0; // счетчики и прокладки
int prev_enc = 0;
int prevcount = 0;
int prev_rpm = 0;
int prev_pwm = 0;
double sumspeed = 0; // прокладка для подсчета средней скорости
int curr = 0;
int counter = 0;

const int ticks_per_rev = 207; // тики энкодера на оборот
const float revn = 2.3; // передаточное отношение моста
int prev_dir = 1; // флаг направления (1 - вперед, -1 - назад)

float rpm_print = 0; // текущий rpm
int current_angle = 0; // текущий угол поворота

float kP = 1; // ПИД-расходники
float kD = 0.1;
float error = 0;
float error_old = 0;
float fix = 0; // сумма ошибки
float coef = 0; // коэффициент умножения PWM двигателя в формате *цель*/*цель+фикс*

int speedArray[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void steering (int turn_angle, Servo drive){ // ïåðåïèñàòü ïîä èíñòà-ðóëåæêó
  if (turn_angle<-30){
    turn_angle = -30;}
  else if (turn_angle>30){
    turn_angle = 30;}
  drive.write(map(turn_angle, -30, 30, 60, 120));
}

void driving (int rpm, int time_accel, int dir){
  if (time_accel<=0){
    time_accel = 1;
  }
  if (prev_rpm != 0){
    if ((prev_dir != dir) or (rpm == 0)){
      for (int i =0; i<5; i++) // плавная остановка за 1 sec
      {
        analogWrite(MOTOR_WRITE, prev_pwm-prev_pwm*(i+1)/5);
        delay(200);
      }
      prev_pwm = 0;
      prev_rpm = 0;
      prev_dir = dir;
    }
  }
  if (dir == 1){
      digitalWrite(DIR_WRITE, HIGH);
      prev_dir = 1;}
  else if (dir == -1)
  {
    digitalWrite(DIR_WRITE, LOW);
    prev_dir = -1;
  }
  if (rpm>MAX_RPM)
  {
    rpm = MAX_RPM;
  }
  int goal_pwm=rpm/2;
  if (rpm != 0 and prev_rpm != rpm){
    int ticks_accel = time_accel*10;
    if (prev_rpm!=0){
      goal_pwm = prev_pwm*rpm/prev_rpm;
    }
    if (goal_pwm>255){goal_pwm=255;}
    for (int j = 0; j<ticks_accel; j++)
    {
      analogWrite(MOTOR_WRITE, (goal_pwm-prev_pwm)*(j+1)/ticks_accel+prev_pwm);
      delay(100);
    }
    prev_pwm = goal_pwm;
    prev_rpm = rpm;
  }
  speed_control();
//  Serial.print(prev_pwm);
//  Serial.print("\n");
  PID(rpm);
}

float speed_control(){
  sumspeed = 0;
  prevcount = (prevcount+1)%10;
  if (enc-prev_enc>-1){
    speedArray[prevcount]=enc-prev_enc;
  }
  prev_enc = enc;
  for (int i = 0; i<10; i++){
      sumspeed += speedArray[i];
//      Serial.print(speedArray[i]);
//      Serial.print(", ");  //- uncomment for controlling speed array
  }
  //Serial.print("\n");
  curr++;
  return rpm_print = sumspeed*60/(2*revn*ticks_per_rev*10*DELAY_TIMER/1000);
//  if (curr*DELAY_TIMER>1000){
//    //Serial.print(analogRead(MOTOR_WRITE));
//    //Serial.print("\n");
//    Serial.print(rpm_print);
//    Serial.print(" rpm or ");
//    Serial.print(rpm_print*0.2512);
//    Serial.print(" m/min, turn angle ");
//    Serial.print(current_angle);
//    Serial.print("\n");
//    curr=0;
//    }
}

int rpm_to_pwm(int rpm_x){
  int pwm_x = (rpm_x + 51.888)/1.7667;  // аппроксимация по экспериментальным данным, пересчитать по факту 1.7667
  return pwm_x;
}

int pwm_to_rpm(int pwm_x){
  int rpm_x = 1.7667*pwm_x - 51.888;
  return rpm_x;
}

void PID(int goal_rpm){
  if (goal_rpm>MAX_RPM){
    goal_rpm = MAX_RPM;
  }
  error = rpm_print - goal_rpm;
  float P = kP*error;
  float D = kD*(error-error_old);
  fix = P+D;
  coef = 1-0.001*fix;
  if (coef<0){coef=goal_rpm;}
//  Serial.print(error);
//  Serial.print(", ");
//  Serial.print(fix);
//  Serial.print(", ");
//  Serial.print(coef);
//  Serial.print("\n");
  volatile int fix_pwm = prev_pwm*coef;
  if (fix_pwm>255){fix_pwm = 255;}
  if (fix_pwm<10){fix_pwm = 10;}
  prev_pwm = fix_pwm;
  analogWrite(MOTOR_WRITE, fix_pwm);
//  Serial.print(fix_pwm);
//  Serial.print("\n");
  delay(DELAY_TIMER);
  error_old = error;
  counter++;
  //prev_rpm = rpm_print;
}

void encoder_counter(){
  enc++;
}
