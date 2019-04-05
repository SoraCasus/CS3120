#include <AFMotor.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

#define PI_32 3.141592654

#define FRONT_SONAR 5
#define RIGHT_SONAR 6
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

#define MOTOR_SPEED 150

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);

LiquidCrystal_I2C lcd(0x27, 16, 2);

typedef uint32_t u32;
typedef double f64;
typedef uint64_t u64;

void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();

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

  motorR.setSpeed(0);
  motorL.setSpeed(0);
}

void loop() {

  int distance = 15;
  bool lastEncoder = LOW;
  while (digitalRead(ROT_ENCODER_SW) != LOW) {
    lcd.setCursor(0, 0);
    lcd.print("Select Distance: ");
    lcd.setCursor(0, 1);
    lcd.print(distance);

    bool encoder = digitalRead(ROT_ENCODER_DT);
    if (encoder != lastEncoder) {
      if (encoder == HIGH) {
        distance += 5;
        if (distance > 60) distance = 15;
        if (distance < 15) distance = 15;

        lastEncoder = encoder;
      }
      lastEncoder = encoder;
    }
  }

  int targetDistance = 0;
  switch(distance) {
    case 20 : { targetDistance = 290; } break;
    case 25 : { targetDistance = 260; } break;
    case 30 : { targetDistance = 230; } break;
    case 35 : { targetDistance = 213; } break;
    case 40 : { targetDistance = 202; } break;
    case 45 : { targetDistance = 190; } break;
    case 50 : { targetDistance = 180; } break;
    case 55 : { targetDistance = 171; } break;
    case 60 : { targetDistance = 167; } break;
  }

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("                ");

  motorR.setSpeed(MOTOR_SPEED);
  motorL.setSpeed(MOTOR_SPEED);
  motorR.run(FORWARD);
  motorL.run(FORWARD);

  while (1) {
    r = digitalRead(R_ENCODER_1);
    l = digitalRead(L_ENCODER_1);

    if (r != rLast) {
      if (r == HIGH) {
        rCounter++;
      }
      rLast = r;
    }

    if (l != lLast) {
      if (l == HIGH) {
        lCounter++;
      }
      lLast = l;
    }

    if (lCounter > rCounter) {
      // motorL.setSpeed(0);
      motorL.run(RELEASE);
    } else if (rCounter > lCounter) {
      // motorR.setSpeed(0);
      motorR.run(RELEASE);
    } else {
      motorL.run(FORWARD);
      motorR.run(FORWARD);
    }

    float dist;
    
    int sharpReading = analogRead(SHARP);

    if (sharpReading >= targetDistance) {
      // Stop the robot
      motorR.run(BACKWARD);
      motorL.run(BACKWARD);
      long lastTime = millis();
      while (millis() - lastTime < 200);
      motorL.setSpeed(0);
      motorR.setSpeed(0);
      motorR.run(RELEASE);
      motorL.run(RELEASE);
      lcd.clear();
      lcd.print(sharpReading);
      return;
    }


  }
}
