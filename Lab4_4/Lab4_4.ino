#include <AFMotor.h>
#include <SD.h>
#include <SPI.h>
#include "LinkedList.h"

#define OPTO_PIN A1
#define CS_PIN 53

#define L_COUNTER 24
#define R_COUNTER 25

File file;

// Todo(Joshua): Move motors to shield pin 3 & 4
AF_DCMotor motorR(1);
AF_DCMotor motorL(2);

LinkedList<int> dataPoints = LinkedList<int>();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(L_COUNTER, INPUT);
  pinMode(R_COUNTER, INPUT);

  InitSD();
  CreateFile("data.csv");

  WriteToFile("FLOOR_SENSOR");
}

void loop() {
  // put your main code here, to run repeatedly:
  bool r = LOW;
  bool l = LOW;
  bool rLast = LOW;
  bool lLast = LOW;
  motorR.setSpeed(150);
  motorL.setSpeed(150);
  motorR.run(FORWARD);
  motorL.run(FORWARD);

  uint32_t lCounter = 0;
  uint32_t rCounter = 0;

  uint64_t timer = millis();

  int i = 0;

  while (millis() - timer < 5000) {
    i++;
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

    //    Serial.print(lCounter);
    //    Serial.print("     ");
    //    Serial.println(rCounter);

    if (i % 10 == 0) {
      dataPoints.add(analogRead(OPTO_PIN));
      i = 0;
    }
  }


  Serial.println("STOPPING");
  motorL.run(BACKWARD);
  motorR.run(BACKWARD);
  delay(100);
  motorL.run(RELEASE);
  motorR.run(RELEASE);

  motorR.setSpeed(1);
  motorL.setSpeed(1);

  int listSize = dataPoints.size();
  for (int i = 0; i < listSize; i++) {
    int val = dataPoints.get(i);
    char c[5];
    String s(val);
    s.toCharArray(c, 5);
    WriteToFile(c);
  }

  delay(1000);
  // Serial.println("Done");

  // motorR.run(RELEASE);
  // motorL.run(RELEASE);
  CloseFile();

  while (true) {
    // motorR.run(RELEASE);
    // motorL.run(RELEASE);

  }
}

void InitSD() {
  Serial.println("Initializing SD Card");
  pinMode(CS_PIN, OUTPUT);
  if (SD.begin()) {
    Serial.println("Initialization Success");
  } else {
    Serial.println("Initialization Fail");
  }
}

void CreateFile(char* fileName) {
  if (SD.exists(fileName)) {
    SD.remove(fileName);
  }

  file = SD.open(fileName, FILE_WRITE);

  if (file) {
    Serial.println("File creation SUCCESS");
    return 1;
  } else {
    Serial.println("File creation FAIL");
    return 0;
  }
}

int WriteToFile(char* text) {
  if (file) {
    file.println(text);
    Serial.println(text);
    return 1;
  } else {
    Serial.println("Failed to write to file");
    return 0;
  }
}

void CloseFile() {
  if (file) {
    file.close();
    Serial.println("Closing file");
  }
}

int OpenFile(char* fileName) {
  file = SD.open(fileName);
  if (file) {
    Serial.println("File opening SUCCESS");
    return 1;
  } else {
    Serial.println("File opening FAIL");
    return 0;
  }
}

String ReadLine() {
  String recieved = "";
  char ch;
  while (file.available()) {
    ch = file.read();
    if (ch == '\n') {
      return String(recieved);
    } else {
      recieved += ch;
    }
  }
  return "";
}
