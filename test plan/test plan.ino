
const int resetPin=8;
int num_turns = 4;

// #include <Wire.h>
#include <VL53L1X.h>
#include <math.h>
#include <time.h>

#define dirPin 10
#define stepPin 11
#define dirPin2 12
#define stepPin2 13
#define speed_rpm 2000

#define LEDpin 5

#define alpha 0.1
#define SIZE 5

VL53L1X gVL531X;
VL53L1X sensor;
VL53L1X sensor2;

void setup() {
  Serial.begin(9600);
 
 // set up time of flight sensors
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

  Serial.println("start first test");

  gVL531X.setTimeout(1000);

  // set up motors

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);

  //set up LED
  pinMode(LEDpin, OUTPUT);

}

void firstTest(){
  double curr_right;
  double init_right_reading;
  
  // Get initial reading
  for(int i=0;i<5;i++){
  init_right_reading=sensor.readRangeContinuousMillimeters();
  }
  Serial.print("initial reading: ");
  Serial.print(init_right_reading);
  Serial.print('\n');

  // get current reading on right tof sensor
  curr_right=init_right_reading;
  Serial.println(curr_right);

  digitalWrite(dirPin, HIGH); // changed this for testing driving straight; original, HIGH both
  digitalWrite(dirPin2, LOW);
  digitalWrite(resetPin,HIGH); 

  // keep moving until distance from right is != inital reading +-2 cm

  while (abs(init_right_reading - curr_right) < 5 ){
    curr_right=sensor.readRangeContinuousMillimeters();
    Serial.print(curr_right);
    for(int i=0;i<50;i++){
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(2000);
    }
  }
  //Shine an LED now

  digitalWrite(LEDpin, HIGH);

}

void thirdTest(){

  double distance;
  distance=sensor2.readRangeContinuousMillimeters();
  Serial.print("initial reading: ");
  Serial.print(distance);
  Serial.print('\n');

  double tileDistToWall = 300;

  digitalWrite(dirPin, HIGH); // changed this for testing driving straight; original, HIGH both
  digitalWrite(dirPin2, LOW);
  digitalWrite(resetPin,HIGH);

  while ( distance > tileDistToWall + 15 ){
    // Serial.println(distance);
    distance=sensor2.readRangeContinuousMillimeters(1);
    for(int i=0;i<50;i++){
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(2000);
    }
  }

  Serial.println("Final Distance");
  Serial.println(distance);

}

void loop(){
  //firstTest();
  //delay(100000);
  // thirdTest()
}



