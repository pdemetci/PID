/* 2015 POE Lab 3 
 *  Pinar Demetci, Yoon-tae Jung, and Inseong Joe

*/

# include <Wire.h>
# include <Adafruit_MotorShield.h>
# include "utility/Adafruit_PWMServoDriver.h"



// ================ Global Objects and Variables ==================== //

// Input Pins
const int analogInPin1 = A0; 
const int analogInPin2 = A1;

// Constants
int startSpeed = 50; // default speed



//Counters
int dir; // counter variable for direction
int current_pos // counter variable for current position of motor

// =========================== Motor Settings ========================== //

// Create motor shield object with default address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create two motors
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);

// ========================= Sensor Methods ============================= //







// ========================= Main Methods =============================== //
void setup() {
  Serial.begin(9600); //start serial connection
  AFMS.begin();
  myMotor1->setSpeed(startSpeed); // Set initial speed of motor 1
  myMotor2->setSpeed(startSpeed); // Set initial speed of motor 2
  dir = FORWARD;

}

void loop() {
  myMotor1->run(dir);
  myMotor2->run(dir);

}
