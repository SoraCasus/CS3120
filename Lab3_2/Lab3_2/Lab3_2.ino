#include <AFMotor.h>
#include <LiquidCrystal_I2C.h>

//#define OPTO_PIN 22


// #define LEFT_SPEED 150
// #define RIGHT_SPEED 150

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define PUSH_BUTTON 15
#define POT_PIN A0
#define L_COUNTER 24
#define R_COUNTER 25

uint32_t rCounter = 0;
uint32_t lCounter = 0;
uint32_t lastTime;
uint32_t l_Count;
uint32_t r_Count;

uint8_t leftSpeed = 150;
uint8_t rightSpeed = 150;


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  pinMode(L_COUNTER, INPUT);
  pinMode(R_COUNTER, INPUT);

  motorR.setSpeed(rightSpeed);
  motorL.setSpeed(leftSpeed);

  lastTime = millis();

}

void loop() {
  // put your main code here, to run repeatedly:

  bool r = LOW;
  bool l = LOW;
  bool rLast = LOW;
  bool lLast = LOW;

  motorR.run(FORWARD);
  motorL.run(FORWARD);
  lastTime = millis();

  while (lCounter < 1627 && rCounter < 1627) {

    r = digitalRead(R_COUNTER);
    // Serial.println(r);
    l = digitalRead(L_COUNTER);
    // Serial.println(l);

    if (r != rLast) {
      if (r == HIGH)   {
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
      motorL.run(RELEASE);
    } else if (rCounter > lCounter) {
      motorR.run(RELEASE);
    } else {
      motorR.run(FORWARD);
      motorL.run(FORWARD);
    }
    
  }

  motorR.run(BACKWARD);
  motorL.run(BACKWARD);
  delay(50);

  motorR.run(RELEASE);
  motorL.run(RELEASE);
  
  lcd.setCursor(0, 0);
  lcd.print("Left  ");
  lcd.print(lCounter);
  lcd.setCursor(0, 1);
  lcd.print("Right ");
  lcd.print(rCounter);


  while (1);

}
