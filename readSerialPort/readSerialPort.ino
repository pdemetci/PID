float kp = 0;
float ki = 0;
float kd = 0;


void setup() {
        Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
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

     
        Serial.print("kp, ki, kd = ");
        Serial.print(kp);
        Serial.print(",");
        Serial.print(ki);
        Serial.print(",");
        Serial.print(kd);

    }
                  
}
