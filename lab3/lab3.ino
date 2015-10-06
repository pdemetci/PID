// ============== Global Variables and Parameters ============= //

// Constant Variables 
const int analogInPin1 = A0;  // Input pins
const int analogInPin2 = A1;
const int setPoint = 400;     // Set wanted default position

// Counters
int lSensor = 0;        // Variable for left sensor value
int rSensor = 0;        // Variable for right sensor value
int lSpeed = 0;         // Variable for left motor speed 
int rSpeed = 0;         // Variable for right motor speed

int errorVal = 0;       // Variable for error value
int prevError = 0;      // Variable for previous error value

// Variables for PID
float kp = 0.00;      // Variables for PID parameters
float kd = 0.00;
float ki = 0.00;
int proportional = 0;
int derivative = 0;
int integral = 0;
int turnVal = 0;      // Variable for total PID value

// Other Variables
int dir;                // Variable for motor direction
int defSpeed = 50;      // Set default speed of motors


// =========================== Motor Settings ========================== //

// Create motor shield object with default address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create two motors
Adafruit_DCMotor *lMotor = AFMS.getMotor(1); //right wheel
Adafruit_DCMotor *rMotor = AFMS.getMotor(2); //left wheel

 // ======================== Other Methods ================== //

// Calculate PID value
void pidCalc(){
  proportional = errorVal;
  integral += proportional;
  derivative = errorVal - prevError;

  turnVal = proportional*kp + integral*ki + derivative*kd;
}

// Calculate sensor value and set speed according to value
void readSensorVal(){
  lSensor = analogRead(analogInPin1);
  rSensor = analogRead(analogInPin2);

// On center
  if (lSensor =< setPoint && rSensor =< setPoint){
    lSpeed = defSpeed;
    rSpeed = defSpeed;
  }

// Tilted left
  if (lSensor =< setPoint && rSensor > setPoint){
    errorVal = rSensor - setPoint;
    lSpeed = defSpeed;
    rSpeed = defSpeed + turnVal;
  }

// Tilted right
  if (lSensor > setPoint && rSensor =< setPoint){
    errorVal = lSensor - setPoint;
    lSpeed = defSpeed + turnVal;
    rSpeed = defSpeed;
    
  }
}

// ===================== Main Methods ======================= //
void setup() {
  Serial.begin(9600);
  AFMS.begin();
  lMotor->setSpeed(defSpeed); // Set initial speed of motor 1
  rMotor->setSpeed(defSpeed); // Set initial speed of motor 2
  dir = FORWARD;

}

void loop() {

// Input values of PID parameters through serial port
  if (Serial.available() > 0){
      for (int x = 0; x<3; x++) {
        switch (x) {
          case 0:
          kp = Serial.parseFloat();
          break;
        case 1:
          ki = Serial.parseFloat();
          break;
        case 2:
          kd = Serial.parseFloat();
          break;
        case 3:
          for (int y = Serial.available(); y == 0; y--){
            Serial.read();
          }
          break;
        }
     }

  // Calculate PID
  pidCalc();

  // Read sensor and set speed
  readSensorVal();

  // Apply speed on motor
  lMotor->setSpeed(lSpeed);
  rMotor->setSpeed(rSpeed);
   
}
