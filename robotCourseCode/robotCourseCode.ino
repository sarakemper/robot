#include <Wire.h>
#include <VL53L1X.h>
#include <math.h>
#include <time.h>
// For the motor:
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


// int resetPin=8;
int num_turns = 4;
unsigned long previousMillis = 0UL;
unsigned long interval = 1000UL;
float Gyro_cal_x, Gyro_cal_y, Gyro_cal_z, Accel_cal_x, Accel_cal_y, Accel_cal_z;
float gpitch=0, gyaw=0, groll=0;
float pitch=0, roll=0, yaw=0;
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float gX, gY, gZ, aX, aY, aZ;
float gXp=0, gYp=0, gZp=0, aXp=0, aYp=0, aZp=0;
int yawp, pitchp, rollp;

void getGyro() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,6,true);  // request a total of 6 registers
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  gX=(GyX*(250.0/32768));
  gY=(GyY*(250.0/32768));
  gZ=(GyZ*(250.0/32768));
}

void getAcc() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,6,true);  // request a total of 6 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  aX=AcX*(2*9.8/32768);
  aY=(AcY*(2*9.8/32768));
  aZ=(AcZ*(2*9.8/32768));
}

void setupGryo() {
  int Gyro_cal_x_sample = 0;
  int Gyro_cal_y_sample = 0;
  int Gyro_cal_z_sample = 0;
  int Accel_cal_x_sample = 0;
  int Accel_cal_y_sample = 0;
  int Accel_cal_z_sample = 0;
  int i;
  delay(5);
  Wire.begin();
  Serial.begin (9600);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(5000);
  int div=100;
  for (i = 0; i <= div; i += 1) {
    getGyro();
    getAcc();
    Gyro_cal_x_sample += gX;
    Gyro_cal_y_sample += gY;
    Gyro_cal_z_sample += gZ;
    Accel_cal_x_sample += aX;
    Accel_cal_y_sample += aY;
    Accel_cal_z_sample += aZ;
    delay(50);
  }
  Gyro_cal_x = Gyro_cal_x_sample / div;
  Gyro_cal_y = Gyro_cal_y_sample / div;
  Gyro_cal_z = Gyro_cal_z_sample / div;
  Accel_cal_x = Accel_cal_x_sample / div;
  Accel_cal_y = Accel_cal_y_sample / div;
  Accel_cal_z = Accel_cal_z_sample / div;
  Serial.println("Calibration Values");
  Serial.print("Gyro_cal_x = ");
  Serial.print(Gyro_cal_x);
  Serial.print("  | Gyro_cal_y = ");
  Serial.print(Gyro_cal_y);
  Serial.print("  | Gyro_cal_z = ");
  Serial.print(Gyro_cal_z);
  Serial.print("  | Accel_cal_x_sample = ");
  Serial.print(Accel_cal_x);
  Serial.print("  | Accel_cal_y_sample = ");
  Serial.print(Accel_cal_y);
  Serial.print("  | Accel_cal_z_sample = ");
  Serial.println(Accel_cal_z);  
}

// SETUP -------------------------------
void setup() {
  pinMode(resetPin,OUTPUT);
  pinMode(sleepPin,OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  digitalWrite(resetPin,HIGH);
  digitalWrite(sleepPin,HIGH);

  setupGryo();

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
  Serial.println("start reading range");
  gVL531X.setTimeout(1000);
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

  while ( fwdDistance > tileDistToWall+5 ){
    // Serial.println(fwdDistance);
    fwdDistance=sensor2.readRangeContinuousMillimeters(1);
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
}

// void turn() {
//   int n=1;
//   while(n){
//     getAcc();
//     getGyro();
//     //Acceleration Calibration
//     if((aX<0&&Accel_cal_x<0)||(aX>0&&Accel_cal_x>0))
//       aX=aX-Accel_cal_x;
//     else
//       aX=aX+Accel_cal_x;
//     if((aY<0&&Accel_cal_y<0)||(aY>0&&Accel_cal_y>0))
//       aY=aY-Accel_cal_y;
//     else
//       aY=aY+Accel_cal_y;
//     if((aZ<0&&Accel_cal_z<0)||(aZ>0&&Accel_cal_z>0))
//       aZ=aZ-Accel_cal_z;
//     else
//       aZ=aZ+Accel_cal_z;
//     //Gyroscope Calibration
//     if((gX<0&&Gyro_cal_x<0)||(gX>0&&Gyro_cal_x>0))
//       gX=gX-Gyro_cal_x;
//     else
//       gX=gX+Gyro_cal_x;
//     if((gY<0&&Gyro_cal_y<0)||(gY>0&&Gyro_cal_y>0))
//       gY=gY-Gyro_cal_y;
//     else
//       gY=gY+Gyro_cal_y;
//     if((gZ<0&&Gyro_cal_z<0)||(gZ>0&&Gyro_cal_z>0))
//       gZ=gZ-Gyro_cal_z;
//     else
//       gZ=gZ+Gyro_cal_z;
//     if(gZ>1.0||gZ<-1.0)
//     gyaw=gyaw+(gZ/50);
//     //Accelerometer
//     roll=asin((((aX))/9.8));
//     pitch=asin((((aY))/9.8));
//     float ro=roll;
//     float pi=pitch;
//     //mapping
//     roll=roll*90.0/1.5;
//     pitch=pitch*90.0/1.5;
//     yaw=gyaw*(90/30);
//     int y=yaw;
//     int p=pitch;
//     int r=roll;
//     Serial.println(yawp);
//     y= (y * alpha) + (yawp* (1.0 - alpha));
//     r = (r * alpha) + (rollp * (1.0 - alpha));
//     p = (p * alpha) + (pitchp * (1.0 - alpha));
//     gXp=gX,gYp=gY,gZp=gZ,aXp=aX,aYp=aY,aZp=aZ;
//     yawp=y;
//     rollp=r;
//     pitchp=p;
//     //delay(50);
//     Serial.print(y);
//     Serial.print("\n");
//     // 90 degrees=1200
//     if(y>-1090){
//       n=1;
//     } else {
//       n=0;
//     }
//     // digitalWrite(dirPin, HIGH);
//     // digitalWrite(dirPin2, HIGH);
//       digitalWrite(dirPin,LOW);
//       digitalWrite(dirPin2, LOW);
//       digitalWrite(sleepPin,HIGH);
//     // Spin the stepper motor 1 revolution slowly:
//       // These four lines result in 1 step:
//       digitalWrite(stepPin, HIGH);
//       digitalWrite(stepPin2, HIGH);
//       delayMicroseconds(700);
//       digitalWrite(stepPin, LOW);
//       digitalWrite(stepPin2, LOW);
//       delayMicroseconds(700);
//   }

//   // TODO: might need to delete/change this after robot turns
//   while(1) {
//     digitalWrite(sleepPin, LOW);
//     delay(5000);
//   }
// }

void turn(int path)
{
  int dist1, dist2,sumDist1=0,sumDist2=0;
  for (int i=0;i<SIZE;i++) {
    sumDist1=sumDist1+sensor.readRangeContinuousMillimeters();
    sumDist2=sumDist2+sensor2.readRangeContinuousMillimeters();
  }
  dist1 = sumDist1/SIZE;
  dist2 = sumDist2/SIZE;
  int n = 1;
  while (n) {
    getAcc();
    getGyro();
    //Acceleration Calibration
    if((aX<0&&Accel_cal_x<0)||(aX>0&&Accel_cal_x>0))
      aX=aX-Accel_cal_x;
    else
      aX=aX+Accel_cal_x;
    if((aY<0&&Accel_cal_y<0)||(aY>0&&Accel_cal_y>0))
      aY=aY-Accel_cal_y;
    else
      aY=aY+Accel_cal_y;
    if((aZ<0&&Accel_cal_z<0)||(aZ>0&&Accel_cal_z>0))
      aZ=aZ-Accel_cal_z;
    else
      aZ=aZ+Accel_cal_z;
    //Gyroscope Calibration
    if((gX<0&&Gyro_cal_x<0)||(gX>0&&Gyro_cal_x>0))
      gX=gX-Gyro_cal_x;
    else
      gX=gX+Gyro_cal_x;
    if((gY<0&&Gyro_cal_y<0)||(gY>0&&Gyro_cal_y>0))
      gY=gY-Gyro_cal_y;
    else
      gY=gY+Gyro_cal_y;
    if((gZ<0&&Gyro_cal_z<0)||(gZ>0&&Gyro_cal_z>0))
      gZ=gZ-Gyro_cal_z;
    else
      gZ=gZ+Gyro_cal_z;
    if(gZ>1.0||gZ<-1.0)
    gyaw=gyaw+(gZ/50);
    //Accelerometer
    roll=asin((((aX))/9.8));
    pitch=asin((((aY))/9.8));
    float ro=roll;
    float pi=pitch;
    //mapping
    roll=roll*90.0/1.5;
    pitch=pitch*90.0/1.5;
    yaw=gyaw*(90/30);
    int y=yaw;
    int p=pitch;
    int r=roll;
    y= (y * alpha) + (yawp* (1.0 - alpha));
    r = (r * alpha) + (rollp * (1.0 - alpha));
    p = (p * alpha) + (pitchp * (1.0 - alpha));
    gXp=gX,gYp=gY,gZp=gZ,aXp=aX,aYp=aY,aZp=aZ;
    yawp=y;
    rollp=r;
    pitchp=p;
    //delay(50);
    Serial.print(y);
    Serial.print("\n");
    double angle = -1075;

    if (path == 2) {
      angle = -1880;
    }
    else if (path == 3) {
      angle = -2710;
    }
    else if (path == 4) {
      angle = -3540;
    }
    else if (path == 5) {
      angle = -4370;
    }
    else if (path == 6) {
      angle = -5200;
    }
    else if (path == 7) {
      angle = -6050;
    }
    else if (path == 8) {
      angle = -6870;
    }
    else if (path == 9) {
      angle = -7710;
    }
    else if (path == 10) {
      angle = -8550;
    }

    if(y>angle){
      n=1;
    } else {
      n=0;
    }
  // digitalWrite(dirPin, HIGH);
  // digitalWrite(dirPin2, HIGH);
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
}

void loop() {

  // for (int i = 0; i+=1; i < 4) {
  //   setupGryo();
  //   turn(1);
  // }
  

  int path = 1;

  while (path <= 11) {
    goStraight(path);
    // setupGryo();
    if (path < 11) {
      turn(path);
    }
    path = path + 1;
    delay(5000);
  }
  
}

  /* TURN ANGLES 
  1050
  1876
  2710
  3536

  4374
  5205
  6042
  6870
  7711
  8533



    turn 2 angle = -1975 
    turn 3 = -2850
    turn 4 = -3740
    turn 5 = -4610
    turn 6 = -5490
    turn 7 = -6374
    turn 8 = -7264
    turn 9 = -8160
    turn 10 = -
    turn 11 = 
    turn 12 =  
  */
