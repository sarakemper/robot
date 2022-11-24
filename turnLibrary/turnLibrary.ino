/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */
#include <VL53L1X.h>
#include <math.h>
#include <time.h>
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

#define dirPin 10
#define stepPin 11
#define dirPin2 12
#define stepPin2 13
#define speed_rpm 2000

#define LEDpin 5

#define alpha 0.1
#define SIZE 5
#define stepsPerRevolution 200
#define resetPin 9
#define sleepPin 8
VL53L1X gVL531X;
VL53L1X sensor;
VL53L1X sensor2;

void setup() {
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  delay(500);
  pinMode(2, INPUT);
  delay(150);
  sensor.init(true);
  delay(100);
  sensor.setAddress((uint8_t)22);
  sensor.setTimeout(500);
  pinMode(3, INPUT);
  delay(150);
  sensor2.init(true);
  delay(100);
  sensor2.setAddress((uint8_t)25);
  sensor2.setTimeout(500);
  Serial.println("addresses set");
  sensor.startContinuous(30);
  sensor2.startContinuous(30);
  Serial.println("start reading range");
  gVL531X.setTimeout(1000);
  pinMode(resetPin,OUTPUT);
  pinMode(sleepPin,OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  digitalWrite(resetPin,HIGH);
  digitalWrite(sleepPin,HIGH);

  Wire.begin();
  
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}

void Moveforward(int setCycle){
     digitalWrite(dirPin, LOW); // changed this for testing driving straight; original, HIGH both
      digitalWrite(dirPin2, HIGH);
      for(int i=0;i<200*setCycle;i++){
      digitalWrite(stepPin, HIGH);
      digitalWrite(stepPin2, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepPin, LOW);
      digitalWrite(stepPin2, LOW);
      delayMicroseconds(1000);
      }
}

void goStraight(int path) {
  double fwdDistance;
  double tileDistToWall = 0;
  fwdDistance=sensor2.readRangeContinuousMillimeters();
  
  Serial.print("initial reading: ");
  Serial.print(fwdDistance);
  Serial.print('\n');
  
  if (path == 1 || path == 2 || path == 3)
    tileDistToWall = 180;
  else if (path == 4 || path == 5 || path == 6 || path == 7)
    tileDistToWall = 400;
  else{
    tileDistToWall = 700;
  }

  digitalWrite(dirPin, LOW); // changed this for testing driving straight; original, HIGH both
  digitalWrite(dirPin2, HIGH);
  // digitalWrite(resetPin,HIGH);
  mpu.update();
  double z = mpu.getAngleZ();
  double x = mpu.getAngleX();
  while ( fwdDistance > tileDistToWall+5 && abs(x)==0){
    // Serial.println(fwdDistance);
    mpu.update();
    x = mpu.getAngleX();
    fwdDistance=sensor2.readRangeContinuousMillimeters();

    for(int i=0;i<50;i++) {
      digitalWrite(stepPin, HIGH);
      digitalWrite(stepPin2, HIGH);
      delayMicroseconds(2000);
      digitalWrite(stepPin, LOW);
      digitalWrite(stepPin2, LOW);
      delayMicroseconds(2000);
      // TODO: add error correction / detection here
    }
  }

  Serial.println("Final fwdDistance");
  Serial.println(fwdDistance);
  if(abs(x)!=0)
  Moveforward(5);
}


void rotation(int angle) {
  
    digitalWrite(8,HIGH);
  mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms
	Serial.print("X : ");
	Serial.print(mpu.getAngleX());
	Serial.print("\tY : ");
	Serial.print(mpu.getAngleY());
	Serial.print("\tZ : ");
	Serial.println(mpu.getAngleZ());
	timer = millis();  
  }
  double z = mpu.getAngleZ();

  while (z > -angle) {
      
    mpu.update();
     z = mpu.getAngleZ();
    Serial.println(z);
    digitalWrite(dirPin,LOW);
    digitalWrite(dirPin2, LOW);
  
  // Spin the stepper motor 1 revolution slowly:
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(400);
    digitalWrite(stepPin, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(400);
  }
    
    }
  void loop(){
    int angle=80;
    for(int i=1;i<7;i++){
    goStraight(i);
    rotation(angle);
    angle=angle+80;
    }
  }