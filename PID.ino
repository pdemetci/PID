/* 2015 POE Lab 3 
 *  Pinar Demetci, Yoon-tae Jung, and Inseong Joe

*/

# include <Wire.h>
# include <Adafruit_MotorShield.h>
# include "utility/Adafruit_PWMServoDriver.h"



// ================ Global Objects and Variables ==================== //

<<<<<<< HEAD
// Input Pins
const int analogInPin1 = A0; 
const int analogInPin2 = A1;

// Variables
int avgSpeed = 255; // default speed

int dir; // variable for direction
int current_pos // variable for current position of motor

int kp;
int kd;
int ki;
=======
// Motor Variables
int startSpeed = 50; // default speed

// Sensor Variables: 
int output1;
int output2;
float sensorVol1;
float sensorVol2;
>>>>>>> FETCH_HEAD

int error;
int lastError;

int proportional;
int derivative;
int integral;

int turnValue;

// =========================== Motor Settings ========================== //

// Create motor shield object with default address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create two motors
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1); //right wheel
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2); //left wheel

// ========================= Sensor Methods ============================= //
void readValues(){
  
}

// ===================== PID ================================ //
void errorCalc(){
  
}

<<<<<<< HEAD

void pidCalc(){
  poportional = error;
  integral += proportional;
  derivative = error - lastError;

  turnValue = proportional*kp + integral*ki + derviative*kd;
}


// ======================= Motor Methods =========================== //
void calcTurn(){  //Restricting the error value between +256.
  if (turnValue< -256){
      turnValue = -256;
    }
  if (turnValue> 256){
      turnValue = 256;
    }
 
// If error_value is less than zero calculate right turn speed values
  if (turnValue< 0){
    rightSpeed = avgSpeed + turnValue;
    leftSpeed = avgSpeed;
    }
 
// Iferror_value is greater than zero calculate left turn values
 
  else{
    rightSpeed = avgSpeed;
    leftSpeed = avgSpeed - turnValue;
    }
}



=======
void runSensor() {
  output1= analogRead(A1);
  output2= analogRead(A0);
  sensorVol1= output1* (5.0/1023.0);
  sensorVol2= output2 * (5.0/1023.0);
  //We may want to return sensorVol values for using it in the loop
}
>>>>>>> FETCH_HEAD

// ========================= Main Methods =============================== //
void setup() {
  Serial.begin(9600); //start serial connection
  AFMS.begin();
  myMotor1->setSpeed(avgSpeed); // Set initial speed of motor 1
  myMotor2->setSpeed(avgSpeed); // Set initial speed of motor 2
  dir = FORWARD;

}
void loop() {
<<<<<<< HEAD
  
=======
  runSensor()
  myMotor1->run(dir);
  myMotor2->run(dir);
>>>>>>> FETCH_HEAD

}
