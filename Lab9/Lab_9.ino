#include <LiquidCrystal_I2C.h>

typedef struct node {
  int value = -1;
  int index = -1;
  struct node* next;
  struct node* prev;
} node_t;

enum class Direction {
  NORTH = 0,
  SOUTH = 1,
  EAST = 2,
  WEST = 3
};

/*  Note:
    Wall = -1
    Empty Space = 0
    Starting Point = -2
    Goal = -3
*/
int plotMap[] = {
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1,  0, -1,  0,  0, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1,
  -1, -1,  0, -2,  0, -1,  0, -3,  0,  0,  0,  0,  0,  0,  0, -1,
  -1,  0, -1,  0,  0, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1,
  -1,  0, -1, -1, -1, -1, -1, -1, -1,  0,  0,  0,  0,  0,  0, -1,
  -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1,
  -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

LiquidCrystal_I2C lcd(0x27, 16, 2);

node_t* headNode;

Direction startDirection = Direction::EAST;
int currentIndex = -1;
int currentX = 0;
int currentY = 0;

void setup() {

  Serial.begin(115200);
  Serial.println("Begin");

  lcd.init();
  lcd.backlight();

  lcd.clear();

  grassFire();

  currentX = currentIndex % 16;
  currentY = currentIndex / 16;

}

void loop() {
  lcd.clear();
  while (digitalRead(6) != LOW) {
    lcd.setCursor(0, 0);
    lcd.print("X=");
    lcd.print(currentX);
    lcd.print(" Y=");
    lcd.print(currentY);
    lcd.print(" Dir:");
    switch (startDirection) {
      case Direction::NORTH: {
          lcd.print("N");
        } break;
      case Direction::EAST: {
          lcd.print("E");
        } break;
      case Direction::WEST: {
          lcd.print("W");
        } break;
      case Direction::SOUTH: {
          lcd.print("S");
        } break;
    }

    lcd.setCursor(0, 1);
    lcd.print("Value:");
    lcd.print(plotMap[currentX + currentY * 16]);

  }

  if (plotMap[currentX + currentY * 16] == 1) {
    // Arrived at destination
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ARRIVED");
    while (1);
  }

  int minValue = 999;
  int dx = 0;
  int dy = 0;
  Direction newDir = Direction::NORTH;

  // North
  if (currentY != 0) {
    if (plotMap[currentIndex - 16] > 0 && plotMap[currentIndex - 16] < minValue) {
      minValue = plotMap[currentIndex - 16];
      dy = -1;
      dx = 0;
      newDir = Direction::NORTH;
    }
  }

  // South
  if (currentY != 7) {
    if (plotMap[currentIndex + 16] > 0 && plotMap[currentIndex + 16] < minValue) {
      minValue = plotMap[currentIndex + 16];
      dy = 1;
      dx = 0;
      newDir = Direction::SOUTH;
    }
  }

  // East
  if (currentX != 15) {
    if (plotMap[currentIndex + 1] > 0 && plotMap[currentIndex + 1] < minValue) {
      minValue = plotMap[currentIndex + 1];
      dy = 0;
      dx = 1;
      newDir = Direction::EAST;
    }
  }

  // West
  if (currentX != 0) {
    if (plotMap[currentIndex - 1] > 0 && plotMap[currentIndex - 1] < minValue) {
      minValue = plotMap[currentIndex - 1];
      dy = 0;
      dx = -1;
      newDir = Direction::WEST;
    }
  }

  if (startDirection != newDir) {
    switch (startDirection) {
      case Direction::NORTH:
        // North
        if (currentY != 0) {
          if (plotMap[currentIndex - 16] > 0 && plotMap[currentIndex - 16] <= minValue) {
            minValue = plotMap[currentIndex - 16];
            dy = -1;
            dx = 0;
            newDir = Direction::NORTH;
          }
        }
        break;
      case Direction::EAST:
        // East
        if (currentX != 15) {
          if (plotMap[currentIndex + 1] > 0 && plotMap[currentIndex + 1] <= minValue) {
            minValue = plotMap[currentIndex + 1];
            dy = 0;
            dx = 1;
            newDir = Direction::EAST;
          }
        }
        break;
      case Direction::SOUTH:
        // South
        if (currentY != 7) {
          if (plotMap[currentIndex + 16] > 0 && plotMap[currentIndex + 16] <= minValue) {
            minValue = plotMap[currentIndex + 16];
            dy = 1;
            dx = 0;
            newDir = Direction::SOUTH;
          }
        }
        break;
      case Direction::WEST:
        // West
        if (currentX != 0) {
          if (plotMap[currentIndex - 1] > 0 && plotMap[currentIndex - 1] <= minValue) {
            minValue = plotMap[currentIndex - 1];
            dy = 0;
            dx = -1;
            newDir = Direction::WEST;
          }
        }
        break;
    }
  }

  currentX += dx;
  currentY += dy;
  currentIndex += (16 * dy);
  currentIndex += dx;
  startDirection = newDir;

  delay(200);

}

void grassFire() {
  bool startExists = false;
  bool endExists = false;
  int startLocation = -3;

  const int l = 128;
  for (int i = 0; i < l; i++) {
    int val = plotMap[i];
    if (val == -2) {
      startExists = true;
      startLocation = i;
      currentIndex = i;
    }
    if (val == -3) endExists = true;
  }

  if (!startExists || !endExists) {
    lcd.setCursor(0, 0);
    if (!startExists) {
      lcd.print("No start exists");
      Serial.println("No start exists");
      lcd.setCursor(0, 1);
    }

    if (!endExists) {
      lcd.print("No end exists");
      Serial.println("No end exists");
    }

    while (true);

  }

  bool change = true;
  int index = 1;
  while (change) {
    change = false;
    for (int i = 0; i < l; i++) {
      if (index == 1) {
        if (plotMap[i] == -3) {

          // Look east
          if (i % 16 != 15) {
            plotMap[i + 1] = index;
            change = true;
          }

          // Look west
          if (i % 16 != 0) {
            plotMap[i - 1] = index;
            change = true;
          }

          // Look north
          if (i - 16 >= 0) {
            plotMap[i - 16] = index;
            change = true;
          }

          // Look south
          if (i + 16 < 128) {
            plotMap[i + 16] = index;
            change = true;
          }
        }
      } else {
        if (plotMap[i] == index - 1) {
          // Look north
          if (i - 16 >= 0) {
            if (plotMap[i - 16] == 0) {
              plotMap[i - 16] = index;
              change = true;
            }
          }

          // Look south
          if (i + 16 < 128) {
            if (plotMap[i + 16] == 0) {
              plotMap[i + 16] = index;
              change = true;
            }
          }

          // Look east
          if (i % 16 != 15) {
            if (plotMap[i + 1] == 0) {
              plotMap[i + 1] = index;
              change = true;
            }
          }

          // Look west
          if (i % 16 != 0) {
            if (plotMap[i - 1] == 0) {
              plotMap[i - 1] = index;
              change = true;
            }
          }
        }
      }
    }
    Serial.print("Iteration = ");
    Serial.println(index);
    for (int i = 0; i < l; i++) {
      if (plotMap[i] < 10 && plotMap[i] >= 0)
        Serial.print(" ");
      Serial.print(plotMap[i]);
      Serial.print(", ");
      if (i % 16 == 15) Serial.println();
    }
    index++;
  }

  // Check if path exists
  bool pathExists = false;
  // Look north
  if (startLocation - 16 >= 0) {
    if (plotMap[startLocation - 16] > 0) pathExists = true;
  }

  // Look south
  if (!pathExists && startLocation + 16 < 128) {
    if (plotMap[startLocation + 16] > 0) pathExists = true;
  }

  // Look east
  if (!pathExists && startLocation % 16 != 15) {
    if (plotMap[startLocation + 1] > 0) pathExists = true;
  }

  // Look west
  if (!pathExists && startLocation % 16 != 0) {
    if (plotMap[startLocation - 1] > 0) pathExists = true;
  }

  if (!pathExists) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("No path exists");
    Serial.println("No path exists");
    return;
  }

}
