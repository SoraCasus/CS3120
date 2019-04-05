

// lab 5 P3 stop when seen a wall on the right side


#include <AFMotor.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <math.h>

#define PI_32 3.141592654

#define SONAR 6
#define R_ENCODER_1 14
#define R_ENCODER_2 15
#define L_ENCODER_1 16
#define L_ENCODER_2 17
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

#define L_COUNTER 24
#define R_COUNTER 25


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

  pinMode(R_ENCODER_1, INPUT);
  pinMode(R_ENCODER_2, INPUT);
  pinMode(L_ENCODER_1, INPUT);
  pinMode(L_ENCODER_2, INPUT);
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

  // lidar.startContinuous(20);

  motorR.setSpeed(85);
  motorL.setSpeed(108);

}

void stopAt() {

  motorR.setSpeed(85);
  motorL.setSpeed(105);

  delay(1000);
  int distance = 500;

  Serial.println("Entered StopAt");

  lidar.startContinuous(50);
  while (1) {

    int reading = lidar.read(true);
    Serial.println(reading);
    if (lidar.timeoutOccurred()) continue;

    if (reading < distance) {

      motorR.run(BACKWARD);
      motorL.run(BACKWARD);

      reading = lidar.read();

      while (reading < distance) {
        motorR.run(BACKWARD);
        motorL.run(BACKWARD);
        delay(50);
        motorR.run(RELEASE);
        motorL.run(RELEASE);
        delay(100);
        reading = lidar.read();
      }
      // lidar.stopContinuous();
      while (1);
    }

  }
  lidar.stopContinuous();

}

void turn() {
  // turns robot 90 degrees.\

  motorR.setSpeed(85);
  motorL.setSpeed(110);

  delay(1000);

  motorR.run(BACKWARD);
  motorL.run(FORWARD);
  delay(820);
  motorR.run(RELEASE);
  motorL.run(RELEASE);
  delay(400);
}

void needToTurn() {

  motorR.run(BACKWARD);
  motorL.run(BACKWARD);

  delay(100);

  motorR.run(RELEASE);
  motorL.run(RELEASE);

  turn();

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

  //goStraight();

  cm = getCm();

  //Serial.println(cm);
  if (cm <= 149 && cm > 10 ) {
    delay(420);
    cm = getCm();
    //Serial.println(cm);
    if (cm <= 149) {
      delay(420);
      cm = getCm();
      //Serial.println(cm);
      if (cm <= 149) {
        needToTurn();
      }
    }

  }

  int reading = lidar.read(true);
  if (lidar.timeoutOccurred()) return;

  if (reading < distance) {
    stopAt();
  }

}
