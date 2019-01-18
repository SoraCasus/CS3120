#include <AFMotor.h>

#define LASER_PIN 14
#define LED_PIN 45
#define OPTO_PIN 22

#define LEFT_SPEED 100
#define RIGHT_SPEED 100

AF_DCMotor motorR(1);
AF_DCMotor motorL(2);
