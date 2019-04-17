#include <Servo.h>

Servo servo;

int data[180];

void setup() {
  Serial.begin(115200);
  servo.attach(9, 300, 2400);
  pinMode(A5, INPUT);
  pinMode(12, OUTPUT);

  digitalWrite(12, HIGH);

  servo.write(0);
  delay(3000);
}

void loop() {

  for (int i = 0; i < 180; i++) {
    servo.write(i);

    int a = analogRead(A5);
    int b = analogRead(A5);
    int c = analogRead(A5);

    int min = min(a, min(b, c));
    int max = max(a, max(b, c));
    int other = a + b + c;

    other -= min;
    other -= max;


    data[i] = other;
    Serial.println(data[i]);

    delay(100);
  }

  for (int i = 179; i >= 0; i--) {
    servo.write(i);

    int a = analogRead(A5);
    int b = analogRead(A5);
    int c = analogRead(A5);

    int min = min(a, min(b, c));
    int max = max(a, max(b, c));
    int other = a + b + c;

    other -= min;
    other -= max;

    int sum = other + data[i];
    data[i] = sum / 2;
    Serial.println(data[i]);
    delay(100);
  }

  int index = 200;
  int min = 1000;
  for (int i = 0; i < 180; i++) {
    if (data[i] < min) {
      min = data[i];
      index = i;
    }
  }

  servo.write(index - 1);

  while (1);

}
