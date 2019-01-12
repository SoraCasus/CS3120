
const uint8_t LASER_PIN = 14;
const uint8_t LED_PIN = 15;
const uint8_t OPTO_PIN = A0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(LASER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(OPTO_PIN, INPUT);

  digitalWrite(LASER_PIN, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:

  int32_t optoValue = analogRead(OPTO_PIN);

  // Todo: Calibrate the Optosensor to detect tape and floor
  // Somewhere between 990 - 1010 is tape
  if(optoValue > 1010) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  Serial.println(optoValue);
  //delay(5);

}
