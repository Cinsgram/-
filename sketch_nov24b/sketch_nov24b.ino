#include <Servo.h>
Servo myservo;
// 5，6，7行代码中的5项数据和14行中的Kp需自行调整
int pos; // 角度
int middle = 103; // 摆正时（黑线在A2处时）舵机转动角度
int max = 127, half_max = 119; // max：黑线在A4处时转动的角度   half_max：黑线在A3处时转动的角度
int min = 30, half_min = 48; // min：黑线在A0处时转动的角度    half_min：黑线在A1处时转动的角度
// A组电机驱动   // Left
int A_PWM = 6; // 控制速度   
int A_DIR = 7; // 控制方向  
// B组电机驱动   // Right
int B_PWM = 5; // 控制速度 
int B_DIR = 4; // 控制方向
float Kp = 35 , Ki = 0, Kd = 0; // pid弯道参数（仅需调Kp）
float previous_error = 0, error = 0, P = 0, I = 0, D = 0, PID_value = 0; // pid参数
int sensor[5] = {0, 1, 2, 3, 4}; // 定义传感器IO口
int sensorReading[5] = {0, 0, 0, 0, 0};
static int initial_motor_speed = 225;

void setup(){
  myservo.attach(9);//数字IO口9号
  myservo.write(middle);
  pinMode(A_DIR, OUTPUT);  
  pinMode(A_PWM, OUTPUT);  
  pinMode(B_DIR, OUTPUT);  
  pinMode(B_PWM, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i < 5; i++) {
    sensorReading[i] = analogRead(sensor[i]);
  } 
  // Serial.print(sensorReading[0]);
  // Serial.print("--");
  // Serial.print(sensorReading[1]);
  // Serial.print("--");
  // Serial.print(sensorReading[2]);
  // Serial.print("--");
  // Serial.print(sensorReading[3]);
  // Serial.print("--");
  // Serial.println(sensorReading[4]);
  int A0_ = sensorReading[0];
  int A1_ = sensorReading[1];
  int A2_ = sensorReading[2];
  int A3_ = sensorReading[3];
  int A4_ = sensorReading[4];
  if(A2_<500){
    Serial.println("保持前行");
    if(pos<middle){
      for(;pos<middle;pos++){
        myservo.write(pos);
      }
    }
    if(pos>middle){
      for(;pos>middle;pos--){
        myservo.write(pos);
      }
    }
  }
  else if(A0_<500){
    Serial.println("车辆左偏，请调整方向");
    for(;pos>=min;pos--){
      myservo.write(pos);
    }
  }
  else if(A1_<500){
    Serial.println("车辆左偏，请调整方向");
    if(pos<half_min){
      for(;pos<half_min;pos++){
        myservo.write(pos);
      }
    }
    if(pos>half_min){
      for(;pos>half_min;pos--){
        myservo.write(pos);
      }
    }
  }
  else if(A4_<500){
    Serial.println("车辆右偏，请调整方向");
    for(;pos<=max;pos++){
      myservo.write(pos);
    }
  }
  else if(A3_<500){
    Serial.println("车辆右偏，请调整方向");
    if(pos<half_max){
      for(;pos<half_max;pos++){
        myservo.write(pos);
      }
    }
    if(pos>half_max){
      for(;pos>half_max;pos--){
        myservo.write(pos);
      }
    }
  }
  else{
    Serial.println("车辆完全偏离黑线，请立即停止");
  }
  read_sensor_values(A0_ , A1_ , A2_ , A3_ , A4_);
  calc_pid();
  motor_control();
  delay(1); // 读取延时
}

void read_sensor_values(int A0_,int A1_,int A2_,int A3_,int A4_)
{
  if (A0_>0&&A0_<500) {
    error = -4;
  } 
  else if (A1_>0&&A1_<500) {
    error = -2;
  }
  else if (A2_>0&&A2_<500) {
    error = 0;
  }
  else if (A3_>0&&A3_<500) {
    error = 2;
  }
   else if (A4_>0&&A4_<500) {
    error = 4;
  } 
}
void calc_pid()
{
  P = error;
  I = I + error;
  D = error - previous_error;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_error = error;
}
void motor_control()
{
  int left_motor_speed ;
  int right_motor_speed ;
  left_motor_speed = initial_motor_speed + PID_value;
  right_motor_speed = initial_motor_speed - PID_value;
  if (left_motor_speed < 0) {
    left_motor_speed = 0;
  }
  if (left_motor_speed > 250) {
    left_motor_speed = 250;
  }
  if (right_motor_speed < 0) {
    right_motor_speed = 0;
  }
  if (right_motor_speed > 250) {
    right_motor_speed = 250;
  }
  motorsWrite(left_motor_speed, right_motor_speed); 
  // Serial.print(left_motor_speed);
  // Serial.print("--");
  // Serial.println(right_motor_speed);
} 

void motorsWrite(int speedL,int speedR)
{
  digitalWrite(A_DIR, HIGH);   
  digitalWrite(B_DIR, HIGH);
  analogWrite(A_PWM, speedL);  
  analogWrite(B_PWM, speedR);
}










