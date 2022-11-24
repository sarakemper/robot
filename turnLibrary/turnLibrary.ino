/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */

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

void setup() {
  Serial.begin(9600);

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

void loop() {
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

  while (z > -90) {
      
    mpu.update();
     z = mpu.getAngleZ();
    Serial.println(z);
    digitalWrite(dirPin,LOW);
    digitalWrite(dirPin2, LOW);
    digitalWrite(8,HIGH);
  // Spin the stepper motor 1 revolution slowly:
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(700);
    digitalWrite(stepPin, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(700);
  }
    digitalWrite(8,LOW);
  }