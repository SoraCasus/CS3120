


#include <AFMotor.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <VL53L1X.h>
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
#define LINE_SENSOR_1 A1
#define LINE_SENSOR_2 A2
#define LINE_SENSOR_3 A3
#define LINE_SENSOR_4 A4
#define LINE_SENSOR_5 A5
#define SHARP A6

#define MOTOR_SPEED 120

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);

VL53L1X lidar;

LiquidCrystal_I2C lcd(0x27, 16, 2);

typedef uint32_t u32;
typedef double f64;
typedef uint64_t u64;

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


  motorR.setSpeed(0);
  motorL.setSpeed(0);
}

void loop() {

  int distance = 25;
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
        if (distance > 150) distance = 150;
        if (distance < 25) distance = 25;

        lastEncoder = encoder;
      }
      lastEncoder = encoder;
    }
  }

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("                ");

  delay(100);

  motorR.setSpeed(95);
  motorL.setSpeed(120);
  motorR.run(FORWARD);
  motorL.run(FORWARD);

  lidar.startContinuous(20);
  while (1) {
    
    int reading = lidar.read(true);
    //Serial.println(reading);
    if (lidar.timeoutOccurred()) continue;
    //reading /= 10;

    if (reading < distance*10) {

      motorR.run(BACKWARD);
      motorL.run(BACKWARD);
      
      motorR.setSpeed(100);
      motorL.setSpeed(120);

      reading = lidar.read();
      reading /= 10;
      while(reading < distance) {
        motorR.run(BACKWARD);
        motorL.run(BACKWARD);
        delay(50);
        motorR.run(RELEASE);
        motorL.run(RELEASE);
        delay(100);
        reading = lidar.read();
        reading /= 10;
      }
      
      break;
    }
  }

  while (1) {
    //Serial.println(lidar.read(true));
  }
  lidar.stopContinuous();
}
