#include <AFMotor.h>


// Note: Using defines for constants to preserve memory usage
#define LASER_PIN 14
#define LED_PIN 45
#define OPTO_PIN 22

#define LEFT_SPEED 80
#define RIGHT_SPEED 60

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);

bool foundLine = false;

int count = 0;

uint32_t lastTime;

void setup() {
  Serial.begin(9600);

  lastTime = millis();

  pinMode(LASER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(OPTO_PIN, INPUT);

  digitalWrite(LASER_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);

  // Give a burst of speed to make sure it actually moves
 
  motorR.setSpeed(80);
  motorL.setSpeed(80);
  motorR.run(FORWARD);
  motorL.run(FORWARD);
  

  delay(50);
}

void loop() {
  int32_t optoValue = digitalRead(OPTO_PIN);
  Serial.println(optoValue);
  while(millis() - lastTime < 5000){
    if( optoValue == LOW ) {
      count++;
      optoValue = digitalRead(OPTO_PIN);
     while(optoValue==LOW) optoValue = digitalRead(OPTO_PIN);;
    }
  }
  motorR.run(RELEASE);
  motorL.run(RELEASE);
  delay(2000);

  for(int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    
  }
  while(1);
}
