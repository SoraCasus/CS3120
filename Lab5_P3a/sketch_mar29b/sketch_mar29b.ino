

// lab 5 P3 stop when seen a wall on the right side


#include <AFMotor.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <math.h>

#define PI_32 3.141592654

#define SONAR 6
#define R_COUNTER_1 18
#define L_COUNTER_1 19
#define I2C_PIN_SDA 20
#define I2C_PIN_SCL 21
#define ROT_ENCODER_CLK 46
#define ROT_ENCODER_SW 47
#define ROT_ENCODER_DT 48
#define SPI_CS_PIN 53
#define POT_PIN A0
#define LINE_SENSOR_1 A1
#define LINE_SENSOR_2 A2
#define LINE_SENSOR_3 A3
#define LINE_SENSOR_4 A4
#define LINE_SENSOR_5 A5
#define SHARP A6

int lCounter = 0;
int rCounter = 0;

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);

VL53L1X lidar;

LiquidCrystal_I2C lcd(0x27, 16, 2);

typedef uint32_t u32;
typedef double f64;
typedef uint64_t u64;

int i = 0;
long timer;

void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();

  Wire.begin();

  lidar.setTimeout(500);
  lidar.init();
  lidar.setDistanceMode(VL53L1X::Long);
  lidar.setMeasurementTimingBudget(50000);

  pinMode(R_COUNTER_1, INPUT);
  pinMode(L_COUNTER_1, INPUT);
  pinMode(ROT_ENCODER_CLK, INPUT);
  pinMode(ROT_ENCODER_SW, INPUT);
  pinMode(ROT_ENCODER_DT, INPUT);
  pinMode(LINE_SENSOR_1, INPUT);
  pinMode(LINE_SENSOR_2, INPUT);
  pinMode(LINE_SENSOR_3, INPUT);
  pinMode(LINE_SENSOR_4, INPUT);
  pinMode(LINE_SENSOR_5, INPUT);
  pinMode(SHARP, INPUT);
  pinMode(SPI_CS_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(L_COUNTER_1), LeftInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(R_COUNTER_1), RightInterrupt, RISING);

  motorR.setSpeed(110);
  motorL.setSpeed(110);

}

void LeftInterrupt() {
  lCounter++;
}
void RightInterrupt() {
  rCounter++;
}

void goStraight() {
  
    if (lCounter > rCounter) {
      motorL.run(RELEASE);
    } else if (rCounter > lCounter) {
      motorR.run(RELEASE);
    } else {
      motorL.run(FORWARD);
      motorR.run(FORWARD);
    }

}
  

void noDelay(int x){
  long lasttime = millis();
  while(millis() - lasttime < x) goStraight(); 
}

void noMove(int x){
  long lasttime = millis();
  while(millis() - lasttime < x) ;
}

void stopAt() {

  motorR.setSpeed(110);
  motorL.setSpeed(110);

  noMove(1000);
  int distance = 499;
  
  lCounter = 0;
  rCounter = 0;

  motorR.run(FORWARD);
  motorL.run(FORWARD);

  goStraight();

  lidar.startContinuous(100);
  while (1) {

    goStraight();

    int reading = lidar.read(true);
    
    if (lidar.timeoutOccurred()) continue;

    if (reading < distance) {

      motorR.run(BACKWARD);
      motorL.run(BACKWARD);

      reading = lidar.read();

      while (reading < distance) {
        motorR.run(BACKWARD);
        motorL.run(BACKWARD);
        noMove(50);
        motorR.run(RELEASE);
        motorL.run(RELEASE);
        noMove(100);
        reading = lidar.read();
      }
      while (1);
    }

  }
  lidar.stopContinuous();

}

void needToTurn() {

  motorR.run(BACKWARD);
  motorL.run(BACKWARD);

  noMove(90);

  motorR.run(RELEASE);
  motorL.run(RELEASE);

  motorR.setSpeed(110);
  motorL.setSpeed(110);

  noMove(1000);

  motorR.run(BACKWARD);
  motorL.run(FORWARD);

  noMove(620); 

  motorR.run(RELEASE);
  motorL.run(RELEASE);
  noMove(1000);

  motorR.run(FORWARD);
  motorL.run(FORWARD);

  stopAt();

}

long getCm() {

  long duration, cm;

  pinMode(SONAR, OUTPUT);
  digitalWrite(SONAR, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR, HIGH);
  delayMicroseconds(5);
  digitalWrite(SONAR, LOW);

  pinMode(SONAR, INPUT);
  duration = pulseIn(SONAR, HIGH);

  return cm = duration / 29 / 2;

}


void loop() {

  long duration, cm;

  int distance = 500;

  motorR.run(FORWARD);
  motorL.run(FORWARD);

  goStraight();

  cm = getCm();

  if (cm <= 149 && cm > 10 ) {
    noDelay(750);
    cm = getCm();
    if (cm <= 149) {
      noDelay(750);
      cm = getCm();
      if (cm <= 149) {
        needToTurn();
      }
    }

  }

  goStraight();

  lidar.startContinuous(100);
  int reading = lidar.read(true);
  if (lidar.timeoutOccurred()) return;

  if (reading < distance) {
    stopAt();
  }

  lidar.stopContinuous();

}
