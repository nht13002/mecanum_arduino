/*
Name:    Vektorrörelse.ino
Created:  3/3/2017 10:37:42 AM
Author: A481830
*/

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#define _USE_MATH_DEFINES

#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


int Feedback1 = 0;
byte receive[6];
int i = 0;
int j = 0;
int Motor1 = 0;
int Motor2 = 0;
int Motor3 = 0;
int Motor4 = 0;

int Dir_1 = 2;
int PWM_1 = 3;
int Brk_1 = 4;

int Dir_2 = 7;
int PWM_2 = 5;
int Brk_2 = 8;

int Dir_3 = 10;
int PWM_3 = 6;
int Brk_3 = 11;

int Dir_4 = 12;
int PWM_4 = 9;
int Brk_4 = 13;

int uno_fake = 2; //these are I2C id
int uno_legit = 3;

double radius = 5; 

long StartTime; 
long ElapsedTime; 
//Det här programmet är till för unon
void setup() {
  // put your setup code here, to run once:
  Wire.begin(1); // join i2c bus (address optional for master)
  Serial.begin(9600);           // start serial for output

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
}

void loop() {
  StartTime = millis();
  if(Serial.available()){
    int command = Serial.parseInt();
    if (command == 1){
      Move(10, 0, 0.5, 0);
    }
    if (command == 2){
      Move(10, 0, 0.5, 0.2);
    }
    if (command == 3){
      Move(10, 0, 0.5, -0.2);
    }
  }
  Stop(100); 
}

void GetMotorValues(int *ReturnArray) {
  Wire.requestFrom(uno_fake, 6);    // request 6 bytes from slave device #2
  i = 0;
  while (Wire.available()) { // slave may send less than requested
    receive[i] = Wire.read();
    i++;
  }
  if (receive[0] == 1 and receive[3] == 2) {
    byte a = receive[1];
    byte b = receive[2];
    Motor1 = a;
    Motor1 = Motor1 << 8 | b;
    ReturnArray[0] = Motor1;
    byte c = receive[4];
    byte d = receive[5];
    Motor2 = c;
    Motor2 = Motor2 << 8 | d;
    ReturnArray[1] = Motor2;
  }

  else{
    Serial.println("FAIL1!");
  }
  //Wire.requestFrom(uno_legit, 6);    // request 3 bytes from slave device #2
  Wire.requestFrom(uno_legit, 6);    // request 6 bytes from slave device #3
  j = 0;
  while (Wire.available()) { // slave may send less than requested
    receive[j] = Wire.read();
    j++;
  }
  if (receive[0] == 3 and receive[3] == 4) {
    byte a = receive[1];
    byte b = receive[2];
    Motor3 = a;
    Motor3 = Motor3 << 8 | b;
    ReturnArray[2] = Motor3;
    byte c = receive[4];
    byte d = receive[5];
    Motor4 = c;
    Motor4 = Motor4 << 8 | d;
    ReturnArray[3] = Motor4;
  }
  else {
    Serial.println("FAIL!");
  }
  
  Serial.println(receive[0]);
  Serial.println(receive[1]); 
  Serial.println(receive[2]); 
  Serial.println(receive[3]); 
}

void Adjust(int *Motorvalues, double *mMotor)
{
  int i = 0; 
  double diff = 0; 
  double desired = 0; 
  double change = 0; 
  int test = 0; 
  while (i < 4) {
    if (abs(mMotor[i]) > abs(mMotor[test])) {
      test = i;
    }
    i++;
  }
  double compvalue = Motorvalues[test] / mMotor[test];
  i = 0; 
  while (i < 4) {
    desired = compvalue * abs(mMotor[i]); 
    diff = Motorvalues[i] - desired;
    if (-1 > diff) {
      mMotor[i] = mMotor[i] * 0.9; 
    }
    else if (diff > 1) {
      mMotor[i] = mMotor[i] * 1.1; 
    }
    i++;
  }
}

double CountDown(int MotorValues[4], double remain) {
  double anglemult = 0.033; 
  double angle_1 = MotorValues[0] * anglemult; 
  double angle_2 = MotorValues[1] * anglemult;
  double angle_3 = MotorValues[2] * anglemult;
  double angle_4 = MotorValues[3] * anglemult;
  double w_1 = double(angle_1 / double(millis()-StartTime));
  double w_2 = double(angle_2 / double(millis() - StartTime));
  double w_3 = double(angle_3 / double(millis() - StartTime));
  double w_4 = double(angle_4 / double(millis() - StartTime));
  double v_x = (w_1 + w_2 + w_3 + w_4)*(radius / 4); 
  double v_y = (-w_1 + w_2 + w_3 - w_4)*(radius / 4);
  double v_r = sqrt(pow(v_x, 2) + pow(v_y, 2));
  double moved = v_r * (millis() - StartTime); 
  StartTime = millis(); 
  remain = remain - moved; 
  return remain; 
}
void SetMotors(double speed, double mMotor[4]) {
  if (speed > 0) {
    digitalWrite(Brk_1, LOW);
    digitalWrite(Brk_2, LOW);
    digitalWrite(Brk_3, LOW);
    digitalWrite(Brk_4, LOW);
    if (mMotor[0] > 0) {
      digitalWrite(Dir_1, LOW);
      analogWrite(PWM_1, 255 * mMotor[0]);
    }
    else {
      mMotor[0] = mMotor[0] * (-1);
      digitalWrite(Dir_1, HIGH);
      analogWrite(PWM_1, 255 * mMotor[0]);
    }
    if (mMotor[1] > 0) {
      digitalWrite(Dir_2, HIGH);
      analogWrite(PWM_2, 255 * mMotor[1]);
    }
    else {
      mMotor[1] = mMotor[1] * (-1);
      digitalWrite(Dir_2, LOW);
      analogWrite(PWM_2, 255 * mMotor[1]);
    }
    if (mMotor[2] > 0) {
      digitalWrite(Dir_3, LOW);
      analogWrite(PWM_3, 255 * mMotor[2]);
    }
    else {
      mMotor[2] = mMotor[2] * (-1);
      digitalWrite(Dir_3, HIGH);
      analogWrite(PWM_3, 255 * mMotor[2]);
    }
    if (mMotor[3] > 0) {
      digitalWrite(Dir_4, HIGH);
      analogWrite(PWM_4, 255 * mMotor[3]);
    }
    else {
      mMotor[3] = mMotor[3] * (-1);
      digitalWrite(Dir_4, LOW);
      analogWrite(PWM_4, 255 * mMotor[3]);
    }
  }
  else if (speed < 0) {
    digitalWrite(Brk_1, LOW);
    digitalWrite(Brk_2, LOW);
    digitalWrite(Brk_3, LOW);
    digitalWrite(Brk_4, LOW);
    if (mMotor[0] > 0) {
      digitalWrite(Dir_1, HIGH);
      analogWrite(PWM_1, 255 * mMotor[0]);
    }
    else {
      mMotor[0] = mMotor[0] * (-1);
      digitalWrite(Dir_1, LOW);
      analogWrite(PWM_1, 255 * mMotor[0]);
    }
    if (mMotor[1] > 0) {
      digitalWrite(Dir_2, LOW);
      analogWrite(PWM_2, 255 * mMotor[1]);
    }
    else {
      mMotor[1] = mMotor[1] * (-1);
      digitalWrite(Dir_2, HIGH);
      analogWrite(PWM_2, 255 * mMotor[1]);
    }
    if (mMotor[2] > 0) {
      digitalWrite(Dir_3, HIGH);
      analogWrite(PWM_3, 255 * mMotor[2]);
    }
    else {
      mMotor[2] = mMotor[2] * (-1);
      digitalWrite(Dir_3, LOW);
      analogWrite(PWM_3, 255 * mMotor[2]);
    }
    if (mMotor[3] > 0) {
      digitalWrite(Dir_4, LOW);
      analogWrite(PWM_4, 255 * mMotor[3]);
    }
    else {
      mMotor[3] = mMotor[3] * (-1);
      digitalWrite(Dir_4, HIGH);
      analogWrite(PWM_4, 255 * mMotor[3]);
    }
  }
}

void Move(int x, int y, double speed, double rotspeed) {
  StartTime = millis(); 
  //Speed är mellan -1 och 1, mMotor kommer hamna mellan -2 och 2 naturligt, normaliseras därför genom /2 
  double angle = atan((double)y / (double)x);
  Serial.print("Angle: ");
  Serial.println(angle);
  double test = angle + ((M_PI) / 4);
  double mMotor[4] = {0, 0, 0, 0};
  mMotor[0] = (speed*sin(test) + rotspeed)/2; 
  mMotor[1] = (speed*cos(test) - rotspeed) / 2;
  mMotor[2] = (speed*cos(test) + rotspeed) / 2;
  mMotor[3] = (speed*sin(test) - rotspeed) / 2;
  Serial.println(mMotor[0]);
  Serial.println(mMotor[1]); 
  Serial.println(mMotor[2]); 
  Serial.println(mMotor[3]); 
  SetMotors(speed, mMotor); 
  double remain = sqrt(pow(x,2) + pow(y,2)); 
  int MotorValues[4];
  while (remain > 0)
  {
    GetMotorValues(&MotorValues[0]);
    Adjust(&MotorValues[0], &mMotor[0]); 
    remain = CountDown(MotorValues, remain); 
    SetMotors(speed, mMotor);
    Serial.println(remain); 
  }
  Stop(0);
}

void Stop(int time) {
  digitalWrite(Dir_1, LOW);
  analogWrite(PWM_1, 0);
  digitalWrite(Brk_1, HIGH);
  digitalWrite(Dir_2, HIGH);
  analogWrite(PWM_2, 0);
  digitalWrite(Brk_2, HIGH);
  digitalWrite(Dir_3, LOW);
  analogWrite(PWM_3, 0);
  digitalWrite(Brk_3, HIGH);
  digitalWrite(Dir_4, HIGH);
  analogWrite(PWM_4, 0);
  digitalWrite(Brk_4, HIGH);
  delay(time);
}
