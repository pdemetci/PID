/* 2015 POE Lab 3 
 *  Pinar Demetci, Yoon-tae Jung, and Inseong Joe

*/

# include <Wire.h>
# include <Adafruit_MotorShield.h>
# include "utility/Adafruit_PWMServoDriver.h"


// ============================= Initial Settings ========================== //

// Initial Constants
int startSpeed = 50;
int dir;


// =========================== Motor Settings ========================== //

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor*myMotor = AFMS.getMotor(1);

// ========================= Main Methods =============================== //
void setup() {
  AFMS.begin();
  myMotor->setSpeed(startSpeed);
  dir = FORWARD;

}

void loop() {
  myMotor->run(dir);

}
