/* 2015 POE Lab 3 
 *  Pinar Demetci, Yoon-tae Jung, and Inseong Joe

*/

# include <Wire.h>
# include <Adafruit_MotorShield.h>
# include "utility/Adafruit_PWMServoDriver.h"



// ================ Global Objects and Variables ==================== //

// Motor Variables
int startSpeed = 50; // default speed

// Sensor Variables: 
int output1;
int output2;
float sensorVol1;
float sensorVol2;

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

void runSensor() {
  output1= analogRead(A1);
  output2= analogRead(A0);
  sensorVol1= output1* (5.0/1023.0);
  sensorVol2= output2 * (5.0/1023.0);
  //We may want to return sensorVol values for using it in the loop
}

// ========================= Main Methods =============================== //
void setup() {
  Serial.begin(9600); //start serial connection
  AFMS.begin();
  myMotor1->setSpeed(startSpeed); // Set initial speed of motor 1
  myMotor2->setSpeed(startSpeed); // Set initial speed of motor 2
  dir = FORWARD;

}
void loop() {
  runSensor()
  myMotor1->run(dir);
  myMotor2->run(dir);

}
