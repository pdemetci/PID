// ============== Global Variables and Parameters ============= //

const int analogInPin1 = A0;
const int analogInPin2 = A1;

const int setPoint = 400;

int lSensor = 0;
int rSensor = 0;

int dir;
int defSpeed = 50;

int lSpeed = 0;
int rSpeed = 0;

int errorVal = 0;
int prevError = 0;

int proportional = 0;
int derivative = 0;
int integral = 0;

int turnVal = 0;

float kp = 0.00;
float kd = 0.00;
float ki = 0.00;

// =========================== Motor Settings ========================== //

// Create motor shield object with default address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create two motors
Adafruit_DCMotor *lMotor = AFMS.getMotor(1); //right wheel
Adafruit_DCMotor *rMotor = AFMS.getMotor(2); //left wheel

 // ======================== Other Methods ================== //

float pidCalc(){
  proportional = errorVal;
  integral += proportional;
  derivative = errorVal - prevError;

  turnVal = proportional*kp + integral*ki + derivative*kd;
}

void readSensorVal(){
  lSensor = analogRead(analogInPin1);
  rSensor = analogRead(analogInPin2);

  if (lSensor =< setPoint && rSensor =< setPoint){
    lSpeed = defSpeed;
    rSpeed = defSpeed;
  }
  if (lSensor =< setPoint && rSensor > setPoint){
    errorVal = rSensor - setPoint;
    lSpeed = defSpeed;
    rSpeed = defSpeed + turnVal;
  }
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
     
  pidCalc();
  readSensorVal();
  lMotor->setSpeed(lSpeed);
  rMotor->setSpeed(rSpeed);
   
}
