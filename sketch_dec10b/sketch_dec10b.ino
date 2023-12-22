#include <Servo.h>
Servo myservo;

int middle = 103; // 黑线在A2处时舵机转动角度，需自设
// A组电机驱动   // Left
int A_PWM = 6; // 控制速度   
int A_DIR = 7; // 控制方向  
// B组电机驱动   // Right
int B_PWM = 5; // 控制速度 
int B_DIR = 4; // 控制方向


void setup() {
  myservo.attach(9);
  myservo.write(middle);
  pinMode(A_DIR, OUTPUT);  
  pinMode(A_PWM, OUTPUT);  
  pinMode(B_DIR, OUTPUT);  
  pinMode(B_PWM, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(A_DIR, HIGH);   
  digitalWrite(B_DIR, HIGH);
  analogWrite(A_PWM, 255);  
  analogWrite(B_PWM, 255);
}
