#include <AFMotor.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <math.h>

#define PI_32 3.141592654

#define DRIVE(x) run((x) ? FORWARD : BACKWARD)
#define DRIVE_INV(x) run((x) ? BACKWARD : FORWARD)
#define BUTTON_WAIT() while(digitalRead(PUSH_BUTTON) == LOW)
#define BETTER_DELAY(x) { long __lastTime__ = millis; while(millis() - __lastTime__ < x); }

#define SERVO_PIN 0
#define FRONT_LASER_PIN 1
#define SONAR_FRONT 5
#define SONAR_RIGHT 6
#define R_ENCODER_1 14
#define R_ENCODER_2 15
#define L_ENCODER_1 16
#define L_ENCODER_2 17
#define ENCODER_INT 18
#define I2C_PIN_SDA 20
#define I2C_PIN_SCL 21
#define PUSH_BUTTON 46
#define SPI_CS_PIN 53
#define POT_PIN A0          // POTENTIOMETER
#define LINE_SENSOR_1 A1
#define LINE_SENSOR_2 A2
#define LINE_SENSOR_3 A3
#define LINE_SENSOR_4 A4
#define LINE_SENSOR_5 A5
#define SHARP A6

#define MOTOR_SPEED 150

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);
Servo servo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

File dataFile;


namespace CS3120 {

void Init() {
    Serial.begin(115200);

    lcd.init();
    lcd.backlight();

    pinMode(SERVO_PIN,OUTPUT);
    pinMode(FRONT_LASER_PIN, OUTPUT);
    pinMode(R_ENCODER_1, INPUT);
    pinMode(R_ENCODER_2, INPUT);
    pinMode(L_ENCODER_1, INPUT);
    pinMode(L_ENCODER_2, INPUT);
    pinMode(ENCODER_INT, INPUT);
    pinMode(PUSH_BUTTON, INPUT);
    pinMode(POT_PIN, INPUT);
    pinMode(LINE_SENSOR_1, INPUT);
    pinMode(LINE_SENSOR_2, INPUT);
    pinMode(LINE_SENSOR_3, INPUT);
    pinMode(LINE_SENSOR_4, INPUT);
    pinMode(LINE_SENSOR_5, INPUT);
    pinMode(SHARP, INPUT);
    pinMode(SPI_CS_PIN, OUTPUT);

    servo.attach(SERVO_PIN);

    motorR.setSpeed(MOTOR_SPEED);
    motorL.setSpeed(MOTOR_SPEED);

    motorR.run(RELEASE);
    motorL.run(RELEASE);

    SD.begin();
    if(SD.exists("data.csv"))
        SD.remove("data.csv");
    
    dataFile = SD.open("data.csv", FILE_WRITE);
    dataFile.println("WALL_READING");

}

uint32_t GetDistance(double distance) {
  if (distance <= 5) {
    return distance * 13.5;
  }
  return (uint32_t)(-55.8 + (16.2 * distance));
}

uint32_t AngleToTicks(uint16_t angle) {
  double distance = PI_32 * 2 * 16.7 * (double)angle;
  distance /= 360;
  double N = 1.04 + (5.56e-3 * angle) + (-1.11e-5 * angle * angle);

  distance /= N;
  distance *= 0.98;

  if (angle > 180) {
    distance *= 1.01;
    if (angle < 270) {
      distance *= 1.02;
    }
  }

  return GetDistance(distance);
}

void DriveDistance(uint16_t dist, bool fwd) {
    bool r = LOW;
    bool l = LOW;
    bool rLast = LOW;
    bool lLast = LOW;

    motorR.DRIVE(fwd);
    motorL.DRIVE(fwd);

    uint32_t target = GetDistance(dist);
    uint32_t lCounter = 0;
    uint32_t rCounter = 0;

    while(lCounter < target && rCounter < target) {
        r = digitalRead(R_ENCODER_1);
        l = digitalRead(L_ENCODER_1);

        if(r != rLast) {
            if(r == HIGH) {
                rCounter++;
            }
            rLast = r;
        }

        if(l != lLast) {
            if(l == HIGH) {
                lCounter++;
            }
            lLast = l;
        }

        if(lCounter > rCounter) {
            motorL.run(RELEASE);
        } else if (rCounter > lCounter) {
            motorR.run(RELEASE);
        } else {
            motorL.DRIVE(fwd);
            motorR.DRIVE(fwd);
        }
    }
    
    motorR.DRIVE_INV(fwd);
    motorL.DRIVE_INV(fwd);

    if(dist > 30) {
        BETTER_DELAY(200);
    } else {
        BETTER_DELAY(50);
    }
    motorR.run(RELEASE);
    motorL.run(RELEASE);
}

}