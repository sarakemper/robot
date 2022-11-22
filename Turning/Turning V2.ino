//For the sensor
#include <Wire.h>
#include <VL53L1X.h>
#include <math.h>
#include <time.h>
#define alpha 0.1
//#define SIZE 1
float Gyro_cal_x, Gyro_cal_y, Gyro_cal_z;
float Accel_cal_x, Accel_cal_y, Accel_cal_z;
float gpitch = 0, gyaw = 0, groll = 0;
float pitch = 0, roll = 0, yaw = 0;
const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float gX, gY, gZ, aX, aY, aZ;
float gXp = 0, gYp = 0, gZp = 0;
float aXp = 0, aYp = 0, aZp = 0;
float yawp, pitchp, rollp;

VL53L1X gVL531X;
VL53L1X sensor;
VL53L1X sensor2;


//Global function variables
float spinning_offset;
//float turn_time_delay = 2000;     //This is the time it will move in between checking yaw when turning
#define LEDpin 5


// For the motor:
#define dirPin 10
#define stepPin 11
#define dirPin2 12
#define stepPin2 13
#define speed_step 500 // this is the amount of milliseconds there shoudl between every stepper motor step
const int sleepPin = 8;
const int resetPin = 7;
const int M2Pin = 9;
float residue_yaw;
int tmp_speed_step;
boolean CheckForCommand;



////Protothreading
//#include <TimedAction.h>
////NOTE: This library has an issue on newer versions of Arduino. After
////      downloading the library you MUST go into the library directory and
////      edit TimedAction.h. Within, overwrite WProgram.h with Arduino.h
////TimedAction
////allows us to set actions to perform on separate timed intervals
////http://playground.arduino.cc/Code/TimedAction
////http://wiring.uniandes.edu.co/source/trunk/wiring/firmware/libraries/TimedAction
//int timerCounter = 0;   // incrementing counter. will crash eventually.
//int timebetweenchecking_motor = 250;




//Protothreading Trile and error
boolean RunMotorsBoolean;
//int millis_counter;
boolean IsSpinning;
boolean InitializeSpinning;
float time_offset;




//#define gyro_pin
#define time_of_flight_sensor_1 2
#define time_of_flight_sensor_2 3




void getGyro()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x43);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 6, true);  // request a total of 6 registers
    GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    gX = (GyX * (250.0 / 32768));
    gY = (GyY * (250.0 / 32768));
    gZ = (GyZ * (250.0 / 32768));
}

void getAcc()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 6, true);  // request a total of 6 registers
    AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    aX = AcX * (2 * 9.8 / 32768);
    aY = (AcY * (2 * 9.8 / 32768));
    aZ = (AcZ * (2 * 9.8 / 32768));
}


void RunMotors(float current_speed_step = 500){
    if (RunMotorsBoolean == true && ((millis() - time_offset) < current_speed_step) && CheckForCommand == true){
        Serial.println("Currently doing first part of motor turning");
        digitalWrite(stepPin, HIGH);
        digitalWrite(stepPin2, HIGH);
        CheckForCommand == false;
    }

    else if (RunMotorsBoolean == true && ((millis() - time_offset) < (2*current_speed_step)) && CheckForCommand == false){
        Serial.println("Currently doing second part of motor turning");
        digitalWrite(stepPin, LOW);
        digitalWrite(stepPin2, LOW);
        RunMotorsBoolean = false;
        CheckForCommand = true;
    }

    else if ((millis() - time_offset) < (3*current_speed_step) && CheckForCommand == true){
        Serial.println("In third");
        RunMotorsBoolean = false;
        time_offset = millis();
    }
    else
        Serial.println("nothing");
}

void Initialize_Spinning(){
    Serial.println("Initialize Spinning");
    spinning_offset = yawp;
    time_offset = millis();
    CheckForCommand = true;
    digitalWrite(sleepPin, HIGH);

}


void Spin90() {
    if (IsSpinning == true){
        if (InitializeSpinning == true){
            Initialize_Spinning();
            InitializeSpinning = false;
        }
        residue_yaw = 90 - spinning_offset + yawp;

        //Turning to Right
        if (residue_yaw > 0){
            digitalWrite(dirPin, LOW);
            digitalWrite(dirPin2, LOW);

            // Big steps
            if (residue_yaw > 1){

                tmp_speed_step = speed_step;
                RunMotorsBoolean = true;

            }

                // Doing small steps and thus decrease speed by increasing step
            else if (residue_yaw > 0.25){
                tmp_speed_step = speed_step * 2;
                RunMotorsBoolean = true;

            }

                // If the robot is inbetween 0.25 degrees, i am fine with it
            else{
                Serial.println("Done rotating");
                IsSpinning = false;
                InitializeSpinning = true;
                digitalWrite(sleepPin, LOW);


                return;
            }
            RunMotors(tmp_speed_step);
        }

            //Turning to Left
        else if (residue_yaw < 0){
            digitalWrite(dirPin, HIGH);
            digitalWrite(dirPin2, HIGH);

            // Big steps
            if (residue_yaw < -1){
                tmp_speed_step = speed_step;
                RunMotorsBoolean = true;

            }

                // Doing small steps and thus decrease speed by increasing step
            else if (residue_yaw < -0.25){
                tmp_speed_step = speed_step * 2;
                RunMotorsBoolean = true;

            }

                // If the robot is inbetween 0.25 degrees, i am fine with it
            else{
                Serial.println("Done rotating");
                IsSpinning = false;
                InitializeSpinning = true;
                digitalWrite(sleepPin, LOW);

                return;
            }
            RunMotors(tmp_speed_step);
        }
    }
}










void Check_TimeOut(){
    if (sensor.timeoutOccurred()){
        Serial.print("sensor1timeout\n");
    }
    if (sensor2.timeoutOccurred()){
        Serial.print("sensor2timeout\n");
    }
}


void UpdateSensorValues(){
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
    float ro = roll;
    float pi = pitch;

    //mapping
    roll = roll * 90.0 / 1.5;
    pitch = pitch * 90.0 / 1.5;
    yaw = gyaw *(90 / 30);
    float y = yaw;
    float p = pitch;
    float r = roll;
    y = (y * alpha) + (yawp* (1.0 - alpha));
    r = (r * alpha) + (rollp * (1.0 - alpha));
    p = (p * alpha) + (pitchp * (1.0 - alpha));
    gXp = gX, gYp = gY, gZp = gZ, aXp = aX, aYp = aY, aZp = aZ;

    yawp = y;
    rollp = r;
    pitchp = p;



//  Serial.print("  | Yaw = "); Serial.print(y);
//  Serial.print("  | Pitch = "); Serial.print(p);
//  Serial.print("  | Roll = ");Serial.println(r);
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
            delayMicroseconds(speed_step);
            digitalWrite(stepPin, LOW);
            digitalWrite(stepPin2, LOW);
            delayMicroseconds(speed_step);
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
            delayMicroseconds(speed_step);
            digitalWrite(stepPin, LOW);
            digitalWrite(stepPin2, LOW);
            delayMicroseconds(speed_step);
        }
    }
    Serial.println("Final Distance");
    Serial.println(distance);
}




















void Initialize_motor(){
    //Motor Stuff:
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(stepPin2, OUTPUT);
    pinMode(dirPin2, OUTPUT);

    pinMode(sleepPin, OUTPUT);
    pinMode(resetPin, OUTPUT);
    pinMode(M2Pin, OUTPUT);

    digitalWrite(resetPin, HIGH);
    digitalWrite(sleepPin, LOW);
    digitalWrite(M2Pin, HIGH);  //Setting step to 1/4

}

void Initialize_gyro(){
    //Gyro
    int Gyro_cal_x_sample = 0;
    int Gyro_cal_y_sample = 0;
    int Gyro_cal_z_sample = 0;
    int Accel_cal_x_sample = 0;
    int Accel_cal_y_sample = 0;
    int Accel_cal_z_sample = 0;
    int i;
    delay(5);
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    delay(5000);
    int div = 100;
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

void Initialize_serial(){
    Serial.begin(9600);
}



//This loop is to initialize the Time of Flight sensor
void Initialize_ToF(){

    pinMode(time_of_flight_sensor_1, OUTPUT);
    pinMode(time_of_flight_sensor_2, OUTPUT);
    digitalWrite(time_of_flight_sensor_1, LOW);
    digitalWrite(time_of_flight_sensor_2, LOW);
    delay(500);

    // Checking Sensor 1 and setting its address
    pinMode(time_of_flight_sensor_1, INPUT);
    delay(150);
    sensor.init(true);
    delay(100);
    sensor.setAddress((uint8_t)22);
    sensor.setTimeout(500);

    // Checking Sensor 2 and setting its address
    pinMode(time_of_flight_sensor_2, INPUT);
    delay(150);
    sensor2.init(true);
    delay(100);
    sensor2.setAddress((uint8_t)25);
    sensor2.setTimeout(500);


    Serial.println("addresses set");
    sensor.startContinuous(30);
    sensor2.startContinuous(30);
    Serial.println("start read range");
    gVL531X.setTimeout(1000);
}


//void Initialize_Protothreading(){
//  //Create a couple timers that will fire repeatedly every x ms
//  //  TimedAction numberThread = TimedAction(700,incrementNumber);
//  //  TimedAction textThread = TimedAction(3000,changeText);
//    TimedAction motorThread = TimedAction(timebetweenchecking_motor, incrementNumber);
//
//
//}



//this is my own version of protothreading
void Initialize_Protothreading(){
    Serial.println("Initilizing Protothreading");
    RunMotorsBoolean = false;
//  millis_counter = millis();
    IsSpinning = true;
    InitializeSpinning = true;
}














void setup() {
    Initialize_serial();
    Initialize_motor();
    Initialize_gyro();
    Initialize_ToF();
    Initialize_Protothreading();
}





void loop() {
    Check_TimeOut();

//Protothreading:
    UpdateSensorValues();
    Spin90();
}




////  Serial.println("Beginning slow clockwise motion");
//  // Set the spinning direction clockwise:
//  digitalWrite(dirPin, HIGH);
//  digitalWrite(dirPin2, LOW);
//
//  // Spin the stepper motor 1 revolution slowly:
////while(1){
//
//    
//
//      // These four lines result in 1 step:
//      digitalWrite(stepPin, HIGH);
//      digitalWrite(stepPin2, HIGH);
//      delayMicroseconds(speed_rpm);
//      digitalWrite(stepPin, LOW);
//      digitalWrite(stepPin2, LOW);
//      delayMicroseconds(speed_rpm);
//




//}



//  int tmp_time = 0;
//  // Spin the stepper motor 1 revolution slowly:
//while(1){
//  for(float i = 1700;i!=500;i--){
//    
//    while(tmp_time !=4000){
//      // These four lines result in 1 step:
//      digitalWrite(stepPin, HIGH);
//      digitalWrite(stepPin2, HIGH);
//      delayMicroseconds(i);
//      digitalWrite(stepPin, LOW);
//      digitalWrite(stepPin2, LOW);
//      delayMicroseconds(i);
////      Serial.println(i);
//      tmp_time ++;
//      
//    }
//    Serial.print("finished round with: ");
//    Serial.println(i);
//  }
//}
//  Serial.println("Ending slow clockwise motion");
//
//  delay(1000);
//
//  
//  Serial.println("Beginning quick counterclockwise motion");
//  // Set the spinning direction counterclockwise:
//  digitalWrite(dirPin, LOW);
//  digitalWrite(dirPin2, LOW);
//
//  
//  // Spin the stepper motor 1 revolution quickly:
//  for (int i = 0; i < stepsPerRevolution; i++) {
//    // These four lines result in 1 step:
//    digitalWrite(stepPin, HIGH);
//    digitalWrite(stepPin2, HIGH);
//    delayMicroseconds(1000);
//    digitalWrite(stepPin, LOW);
//    digitalWrite(stepPin2, LOW);
//    delayMicroseconds(1000);
//  }
//  Serial.println("Ending quick counterclockwise motion");
//
//  delay(1000);
//
//
//  Serial.println("Beginning quick clockwise (5 rotation) motion");
//  // Set the spinning direction clockwise:
//  digitalWrite(dirPin, HIGH);
//  digitalWrite(dirPin2, HIGH);
//
//  // Spin the stepper motor 5 revolutions fast:
//  for (int i = 0; i < 5 * stepsPerRevolution; i++) {
//    // These four lines result in 1 step:
//    digitalWrite(stepPin, HIGH);
//    digitalWrite(stepPin2, HIGH);
//    delayMicroseconds(500);
//    digitalWrite(stepPin, LOW);
//    digitalWrite(stepPin2, LOW);
//    delayMicroseconds(500);
//  }
//  Serial.println("Ending quick clockwise (5 rotation) motion");
//
//
//  delay(1000);
//
//
//  Serial.println("Beginning quick counterclockwise (5 rotation) motion");
//  // Set the spinning direction counterclockwise:
//  digitalWrite(dirPin, LOW);
//  digitalWrite(dirPin2, LOW);
//
//  //Spin the stepper motor 5 revolutions fast:
//  for (int i = 0; i < 5 * stepsPerRevolution; i++) {
//    // These four lines result in 1 step:
//    digitalWrite(stepPin, HIGH);
//    digitalWrite(stepPin2, HIGH);
//    delayMicroseconds(500);
//    digitalWrite(stepPin, LOW);
//    digitalWrite(stepPin2, LOW);
//    delayMicroseconds(500);
//  }
//  Serial.println("Ending quick counterclockwise (5 rotation) motion");
//
//  delay(1000);




//void SpinDegrees(int tmp_degrees) {
//  float* test;
////  test = GetSensorValues();
////  spinning_offset = test[0];
//  spinning_offset = GetSensorValues()[0];

//void SpinDegrees(int desired_degrees) {
////  float* test;
////  test = GetSensorValues();
////  spinning_offset = test[0];
//  UpdateSensorValues();
//  spinning_offset = yawp;
////  Serial.print("Spinning Offset: ");Serial.println(spinning_offset);
//
//  float time_offset;
//  float residue_yaw;
//  int tmp_speed_step;
//  
//  
//  while(true){
//    //Finding out relative yaw at that time
//    UpdateSensorValues();
//    residue_yaw = desired_degrees - spinning_offset + yawp;
//    time_offset = millis();
//    Serial.println();
//
//    //Turning to Right
//    if (residue_yaw > 0){
//      digitalWrite(dirPin, LOW);
//      digitalWrite(dirPin2, LOW);
//      
//      // Big steps
//      if (residue_yaw > 1){
//        tmp_speed_step = speed_step;
//      }
//      
//      // Doing small steps and thus decrease speed by increasing step
//      else if (residue_yaw > 0.25){
//        tmp_speed_step = speed_step * 2;
//      }
//
//      // If the robot is inbetween 0.25 degrees, i am fine with it
//      else{
//        Serial.println("Done rotating");  
//        return;
//      }
//      delay(100);
////      while((turn_time_delay + time_offset) > millis()){ 
////        digitalWrite(stepPin, HIGH);
////        digitalWrite(stepPin2, HIGH);
////        delayMicroseconds(tmp_speed_step);
////        digitalWrite(stepPin, LOW);
////        digitalWrite(stepPin2, LOW);
////        delayMicroseconds(tmp_speed_step);
////        Serial.print(" Turning Right ");
////      }
//    }
//    
//    //Turning to Left
//    else if (residue_yaw < 0){
//      digitalWrite(dirPin, HIGH);
//      digitalWrite(dirPin2, HIGH);
//
//      // Big steps
//      if (residue_yaw < -1){
//        tmp_speed_step = speed_step;
//      }
//      
//      // Doing small steps and thus decrease speed by increasing step
//      else if (residue_yaw < -0.25){
//        tmp_speed_step = speed_step * 2;
//      }
//
//      // If the robot is inbetween 0.25 degrees, i am fine with it
//      else{
//        Serial.println("Done rotating");  
//        return;
//      }
//
//      
//      while((turn_time_delay + time_offset) > millis()){ 
//        digitalWrite(stepPin, HIGH);
//        digitalWrite(stepPin2, HIGH);
//        delayMicroseconds(tmp_speed_step);
//        digitalWrite(stepPin, LOW);
//        digitalWrite(stepPin2, LOW);
//        delayMicroseconds(tmp_speed_step);
//        Serial.print("Turning Left ");
//      }
//    }
//      
//    
//    
//  }
//
//  
////  if (tmp_degrees < 0){
////    digitalWrite(dirPin, HIGH);
////    digitalWrite(dirPin2, HIGH);
////
////    while((turn_time_per_degree * (-tmp_degrees) + tmp_time) > millis()){ 
////        // These four lines result in 1 step:
////        digitalWrite(stepPin, HIGH);
////        digitalWrite(stepPin2, HIGH);
////        delayMicroseconds(speed_rpm);
////        digitalWrite(stepPin, LOW);
////        digitalWrite(stepPin2, LOW);
////        delayMicroseconds(speed_rpm);
////        Serial.println("Turn LEFT");
//////        Serial.print(millis());
////    }
////  }
////  else{
////    digitalWrite(dirPin, LOW);
////    digitalWrite(dirPin2, LOW);
////    while((turn_time_per_degree * tmp_degrees + tmp_time) > millis()){ 
////      digitalWrite(stepPin, HIGH);
////      digitalWrite(stepPin2, HIGH);
////      delayMicroseconds(speed_rpm);
////      digitalWrite(stepPin, LOW);
////      digitalWrite(stepPin2, LOW);
////      delayMicroseconds(speed_rpm);
////      Serial.println("Turn RIGHT");
//////      Serial.print(millis());
////  }
//// }
// 
//
////  if (tmp_degrees < 0){
////    digitalWrite(dirPin, HIGH);
////    digitalWrite(dirPin2, HIGH);
//    
////    for (float yaw = 0;-yaw > tmp_degrees; yaw = yawp - spinning_offset){
////        // These four lines result in 1 step:
////        digitalWrite(stepPin, HIGH);
////        digitalWrite(stepPin2, HIGH);
////        delayMicroseconds(speed_rpm);
////        digitalWrite(stepPin, LOW);
////        digitalWrite(stepPin2, LOW);
////        delayMicroseconds(speed_rpm);
////        Serial.println("Turn LEFT");
////        UpdateSensorValues();
////    }
////  }
////  else{
////      digitalWrite(dirPin, LOW);
////      digitalWrite(dirPin2, LOW);
////        for (float yaw = 0;-yaw < tmp_degrees; yaw = yawp - spinning_offset){
////        // These four lines result in 1 step:
////        digitalWrite(stepPin, HIGH);
////        digitalWrite(stepPin2, HIGH);
////        delayMicroseconds(speed_rpm);
////        digitalWrite(stepPin, LOW);
////        digitalWrite(stepPin2, LOW);
////        delayMicroseconds(speed_rpm);
////        Serial.println("Turn RIGHT");
////        UpdateSensorValues();
////    }
////  }
////  Serial.println("Done rotating");  
//}