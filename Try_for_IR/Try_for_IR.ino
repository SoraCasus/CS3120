// the setup routine runs once when you press reset:
#include <AFMotor.h>

#define PHOTOCELL A15
#define IRSENSOR_L 38
#define IRSENSOR_R 39
#define R_COUNTER_1 18
#define L_COUNTER_1 19
#define FRONT_SONAR 5
#define RIGHT_SONAR 6

int front;
int right;
float fLim = 20;
float rLim = 25;

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);


void setup() {
  // initialize serial communication at 9600 bits per second:

  Serial.begin(115200);
  pinMode(R_COUNTER_1, INPUT);
  pinMode(L_COUNTER_1, INPUT);
  //pinMode(PHOTOCELL, INPUT);
  pinMode(IRSENSOR_L, INPUT);
  pinMode(IRSENSOR_R, INPUT);
  pinMode(1, OUTPUT);
  motorR.setSpeed(80);
  motorL.setSpeed(100);
  
}


long get_rCm() {

  long duration, cm;

  pinMode(RIGHT_SONAR, OUTPUT);
  digitalWrite(RIGHT_SONAR, LOW);
  delayMicroseconds(2);
  digitalWrite(RIGHT_SONAR, HIGH);
  delayMicroseconds(5);
  digitalWrite(RIGHT_SONAR, LOW);

  pinMode(RIGHT_SONAR, INPUT);
  duration = pulseIn(RIGHT_SONAR, HIGH);

  return cm = duration / 29 / 2;

}

long get_fCm() {

  long duration, cm;

  pinMode(FRONT_SONAR, OUTPUT);
  digitalWrite(FRONT_SONAR, LOW);
  delayMicroseconds(2);
  digitalWrite(FRONT_SONAR, HIGH);
  delayMicroseconds(5);
  digitalWrite(FRONT_SONAR, LOW);

  pinMode(FRONT_SONAR, INPUT);
  duration = pulseIn(FRONT_SONAR, HIGH);

  return cm = duration / 29 / 2;

}

void avoidObstacle(){
      
      motorR.setSpeed(95);
      motorL.setSpeed(120);
      
      motorR.run(BACKWARD);
      motorL.run(BACKWARD);
      delay(100);
      motorL.run(BACKWARD);
      motorR.run(FORWARD);
      
      delay(800);

      motorR.run(RELEASE);
      motorL.run(RELEASE);
      delay(1000);

      motorR.setSpeed(70);
      motorL.setSpeed(100);
      motorR.run(FORWARD);
      motorL.run(FORWARD);
      delay(1000);

      while(get_rCm() <= rLim){
         
      }
      
      delay(2000);

      motorR.run(BACKWARD);
      motorL.run(FORWARD);
      
      while(!(digitalRead(IRSENSOR_L) == 0 && digitalRead(IRSENSOR_R) == 0)){
        
      }

      motorR.run(FORWARD);
      motorL.run(FORWARD);
      
}

void loop() {
  
  motorR.run(FORWARD);
  motorL.run(BACKWARD);
  
  int sensorValue = analogRead(A15);

  while(!(digitalRead(IRSENSOR_L) == 0 && digitalRead(IRSENSOR_R) == 0)){
    
  }

  motorR.run(BACKWARD);
  motorL.run(FORWARD);
  delay(80);

  motorR.run(FORWARD);
  
  digitalWrite(1, HIGH);

  while(1){
    if(digitalRead(IRSENSOR_L) != 0){
      motorL.setSpeed(150);
      motorR.setSpeed(80);
    }
    if(digitalRead(IRSENSOR_R) != 0){
      motorL.setSpeed(100);
      motorR.setSpeed(130);
    }
    
    if(digitalRead(IRSENSOR_R) == 0 && digitalRead(IRSENSOR_L) ==0 ){
      motorL.setSpeed(90);
      motorR.setSpeed(70);
    }

    
    if(get_fCm() <= fLim){
      avoidObstacle();
    }
    else delay(30);
    
    if(analogRead(A15) >= 620){
      motorR.run(RELEASE);
      motorL.run(RELEASE);
      while(1);
    }
    
  }
  
}
