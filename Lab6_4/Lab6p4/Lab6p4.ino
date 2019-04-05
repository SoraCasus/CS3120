
#include <AFMotor.h>

#define LEFT_SENSOR A1
#define INNER_LEFT_SENSOR A2
#define CENTER_SENSOR A3
#define INNER_RIGHT_SENSOR A4
#define RIGHT_SENSOR A5

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);

int motorSpeed = 100;
int x = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(INNER_LEFT_SENSOR, INPUT);
  pinMode(CENTER_SENSOR, INPUT);
  pinMode(INNER_RIGHT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  motorR.setSpeed(motorSpeed-20);
  motorL.setSpeed(motorSpeed);

  motorR.run(RELEASE);
  motorL.run(RELEASE);

}


/*    * /
void findLine(){
  
    while( analogRead(CENTER_SENSOR) > 700 ){
    
    if((analogRead(LEFT_SENSOR) < 700) || (analogRead(INNER_LEFT_SENSOR) < 700)){
      while(analogRead(CENTER_SENSOR) > 700){
        motorR.run(FORWARD);
        motorL.run(RELEASE);
      }
    } else if((analogRead(RIGHT_SENSOR) < 700) || (analogRead(INNER_RIGHT_SENSOR) < 700)){
      while(analogRead(CENTER_SENSOR) > 700){
        motorR.run(RELEASE);
        motorL.run(FORWARD);
      }
    } else{
      
       
       }
    
  }
  
}

*/

void followLine(){

    
    if(analogRead(INNER_LEFT_SENSOR) < 700){
      
      motorR.run(FORWARD);
      motorL.run(RELEASE);
    }
    else if(analogRead(INNER_RIGHT_SENSOR) < 700){

      motorR.run(RELEASE);
      motorL.run(FORWARD);
    }
    else if(analogRead(CENTER_SENSOR) < 700) {

      motorR.run(FORWARD);
      motorL.run(FORWARD);
    }

}

void loop() {
  // put your main code here, to run repeatedly:

  followLine();
  
  if(x >=1 && analogRead(LEFT_SENSOR) < 700 && analogRead(RIGHT_SENSOR) < 700){
    
    motorR.run(FORWARD);
    motorL.run(FORWARD);
/*
      long lastTime = millis();
      while (millis() - lastTime < 180){
        if(analogRead(LEFT_SENSOR) > 700 || analogRead(RIGHT_SENSOR) > 700){
          motorR.run(BACKWARD);
          motorL.run(BACKWARD);
          delay(200);
          motorR.run(RELEASE);
          motorL.run(RELEASE);
        }
      }
*/
    delay(500);
    
    motorR.run(RELEASE);
    motorL.run(RELEASE);

    while(true);
  }

  if(analogRead(LEFT_SENSOR)<700 && analogRead(RIGHT_SENSOR)<700){

    x++;

    motorR.run(BACKWARD);
    motorL.run(BACKWARD);

    delay(500);
    
    motorR.run(FORWARD);
    motorL.run(BACKWARD);

    
    //while(analogRead(CENTER_SENSOR) < 700){  };

    delay(300);

    while(analogRead(CENTER_SENSOR) > 700){  };

    motorR.run(FORWARD);
    motorL.run(FORWARD);

   followLine();
    
  }

}
