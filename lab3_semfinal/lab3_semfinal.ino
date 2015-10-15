#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// ============== Global Variables and Parameters ============= //

// Constant Variables 
const int analogInPin1 = A0;  // Input pins
const int analogInPin2 = A1;
const int setPoint = 800;     // Set wanted default position

// Counters
int lSensor = 0;        // Variable for left sensor value
int rSensor = 0;        // Variable for right sensor value
int lSpeed = 0;         // Variable for left motor speed 
int rSpeed = 0;         // Variable for right motor speed

int dif = 0;            // Variable for differense between sensor values
int prevProportional = 0;

int errorVal = 0;       // Variable for error value
int prevError = 0;      // Variable for previous error value

// Variables for PID
float kp = 0.05;      // Variables for PID parameters
float kd = 0.005;
float ki = 0.00;
float blank = 0;
int proportional = 0;
int derivative = 0;
int integral = 0;
int turnVal = 0;      // Variable for total PID value

// Other Variables
int dir;                // Variable for motor direction
int maxSpeed = 255;
int defSpeed = 15;      // Set default speed of motors

int prevTime = 0;
int dt = 0;

// =========================== Motor Settings ========================== //

// Create motor shield object with default address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create two motors
Adafruit_DCMotor *lMotor = AFMS.getMotor(4); //right wheel
Adafruit_DCMotor *rMotor = AFMS.getMotor(3); //left wheel

 // ======================== Other Methods ================== //

// Calculate PID value
void pidCalc(){
  proportional = abs(dif);
  integral += proportional*dt;
  derivative = abs(proportional - prevProportional)/dt;

  prevProportional = proportional;
  
  turnVal = proportional*kp + integral*ki + derivative*kd;
}

// Calculate sensor value and set speed according to value
void readSensorVal(){
  lSensor = analogRead(analogInPin1);
  rSensor = analogRead(analogInPin2);
  dif = lSensor - rSensor;


  if (abs(dif) <= 10){
    lSpeed = defSpeed;
    rSpeed = defSpeed;
  }
  else{
    if (dif > 0){
      lSpeed = defSpeed;
      if(defSpeed + turnVal <= 255){
      rSpeed = defSpeed + turnVal;
    }
      else{
        rSpeed = 255;
      }
    }
    if (dif < 0){
      rSpeed = defSpeed;
      if(defSpeed + turnVal <= 255){
      lSpeed = defSpeed + 1.2*turnVal;
    }
      else{
        lSpeed = 255;
      }
    }
  }

}

  
//// On center
//  if (lSensor <= setPoint && rSensor <= setPoint){
//    lSpeed = defSpeed;
//    rSpeed = defSpeed;
//  }
//
//// Tilted left
//  if (lSensor <= setPoint && rSensor > setPoint){
//    errorVal = abs(rSensor - setPoint);
//    rSpeed = defSpeed;
//    if (defSpeed + turnVal <= 255){
//      lSpeed = abs(defSpeed + turnVal);
//    }
//    else{
//      lSpeed = 255;
//    }
//  }
//
//// Tilted right
//  if (lSensor > setPoint && rSensor <= setPoint){
//    errorVal = abs(lSensor - setPoint);
//    lSpeed = defSpeed;
//    if (defSpeed + turnVal <= 255){
//      rSpeed = abs(defSpeed + turnVal);
//    }
//    else{
//      rSpeed = 255;
//    }
//    
//  }


// ===================== Main Methods ======================= //
void setup() {
  Serial.begin(9600);
  AFMS.begin();
  lMotor->setSpeed(defSpeed); // Set initial speed of motor 1
  rMotor->setSpeed(defSpeed); // Set initial speed of motor 2
  dir = BACKWARD;
}

void loop() {
 //Input values of PID parameters through serial port
  if (Serial.available() > 0){
    kp = Serial.parseFloat();
    ki = Serial.parseFloat();
    kd = Serial.parseFloat();
    blank = Serial.parseFloat();
//      for (int x = 0; x<3; x++) {
//        switch (x) {
//          case 0:
//          kp = Serial.parseFloat();
//          break;
//        case 1:
//          ki = Serial.parseFloat();
//          break;
//        case 2:
//          kd = Serial.parseFloat();
//          break;
//        case 3:
//          for (int y = Serial.available(); y == 0; y--){
//            Serial.read();
//          }
//          break;
//        }
//     }
  }


  Serial.print("kp,ki,kd: ");
  Serial.print(kp,4);
  Serial.print(",");
  Serial.print(ki,5);
  Serial.print(",");
  Serial.print(kd,4);
  
  // Run motors
  lMotor->run(dir);
  rMotor->run(dir);
  // Calculate PID
  pidCalc();
  
  // Read sensor and set speed
  readSensorVal();
  // Apply speed on motor
  lMotor->setSpeed(lSpeed);
  rMotor->setSpeed(rSpeed);


  Serial.print("Sensor Values: ");
  Serial.print(lSensor);
  Serial.print(",");
  Serial.print(rSensor);
  Serial.print("/");
  Serial.print("Speed values: ");
  Serial.print(lSpeed);
  Serial.print(",");
  Serial.println(rSpeed);

  int currTime = millis();
  dt = currTime - prevTime;
  prevTime = currTime;

}
