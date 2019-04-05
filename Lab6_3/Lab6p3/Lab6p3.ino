#include <AFMotor.h>

#define BOX_SENSOR A1
#define LINE_SENSOR A5

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);

int motorSpeed = 100;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  pinMode(BOX_SENSOR, INPUT);
  pinMode(LINE_SENSOR, INPUT);
  
  motorR.setSpeed(motorSpeed);
  motorL.setSpeed(motorSpeed);

  motorR.run(FORWARD);
  motorL.run(FORWARD);

}

void loop() {

      while( analogRead(BOX_SENSOR) > 700) {
        
        if( analogRead(LINE_SENSOR) < 700) {
          
          motorL.run(RELEASE);
          motorR.run(FORWARD);
          
        } else {
          
           motorR.run(RELEASE);
           motorL.run(FORWARD);
          
        }
        
      }
      
      motorL.run(RELEASE);
      motorR.run(RELEASE);

      while(1);
  
}
