
class Node {
  public int value;
  public boolean wall;
  public boolean goal;
  public boolean start;
  public boolean path;

  public Node() {
    this.value = -1;
    this.wall = false;
    this.path = false;
    this.goal = false;
    this.start = false;
  }
}

enum SelectMode {
  TOGGLE_WALL, 
    SET_START, 
    SET_GOAL
}

enum Direction {
  NORTH, 
    SOUTH, 
    EAST, 
    WEST
}

private static Node[][] map;
private static SelectMode selectMode;
private static boolean lastMouse;
private static boolean lastSpace;

void setup() {
  size(800, 600);
  map = new Node[10][10];
  selectMode = SelectMode.TOGGLE_WALL;
  lastMouse = false;
  lastSpace = false;

  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      map[j][i] = new Node();
      if (i == 0 || i == 9 || j == 0 || j == 9) {
        map[j][i].wall = true;
      }
    }
  }
}

void draw() {
  clear();
  if (mousePressed != lastMouse && mousePressed) {
    if (mouseX > 0 && mouseX < 500 && mouseY > 0 && mouseY < 500) {
      // Determine which node is clicked
      int xpos = (int)(mouseX / 50);
      int ypos = (int)(mouseY / 50);

      Node n = map[xpos][ypos];
      switch(selectMode) {
      case TOGGLE_WALL: 
        {
          n.wall = !n.wall;
          n.goal = false;
          n.start = false;
        } 
        break;
      case SET_START: 
        {
          for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 10; j++) {
              map[j][i].start = false;
            }
          }
          n.wall = false;
          n.goal = false;
          n.start = true;
          // Todo: Make sure there's only one start point
        } 
        break;
      case SET_GOAL: 
        {
          for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 10; j++) {
              map[j][i].goal = false;
            }
          }
          n.wall = false;
          n.goal = true;
          n.start = false;
          // Todo: Make sure there's only one goal point
        } 
        break;
      }
    }
  }

  lastMouse = mousePressed;

  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      Node n = map[j][i];
      stroke(0);
      int x = j * 50;
      int y = i * 50;
      if (n.wall) {
        fill(204, 102, 0);
        rect(x, y, 50, 50);
      } else if (n.goal) {
        fill(255, 0, 0);
        rect(x, y, 50, 50);
      } else if (n.start) {
        fill(0, 255, 0);
        rect(x, y, 50, 50);
      } else if (n.path) {
        fill(128, 255, 128);
        rect(x, y, 50, 50);
        textAlign(CENTER, CENTER);
        fill(0);
        text(n.value, x + 25, y + 25);
      } else {
        fill(255);
        rect(x, y, 50, 50);
        textAlign(CENTER, CENTER);
        fill(0);
        text(n.value, x + 25, y + 25);
      }
    }
  }

  switch(selectMode) {
  case TOGGLE_WALL: 
    {
      fill(255);
      text("TOGGLE WALL", 600, 500);
    }
    break;
  case SET_START: 
    {
      fill(255);
      text("SET START", 600, 500);
    } 
    break;
  case SET_GOAL: 
    {
      fill(255);
      text("SET GOAL", 600, 500);
    } 
    break;
  }
}

void keyPressed() {
  boolean space = (key == ' ');
  if (space != lastSpace && space) {
    if (selectMode == SelectMode.TOGGLE_WALL) {
      selectMode = SelectMode.SET_START;
    } else if (selectMode == SelectMode.SET_START) {
      selectMode = SelectMode.SET_GOAL;
    } else if (selectMode == SelectMode.SET_GOAL) {
      selectMode = SelectMode.TOGGLE_WALL;
    }
  }

  if (key == 'p' || key == 'P') {
    // Execute the algorithm
    grassFire();
  }
}

void grassFire() {
  boolean startExists = false;
  boolean endExists = false;
  int startX = 0;
  int startY = 0;
  int endX = 0;
  int endY = 0;

  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      Node n = map[j][i];
      if (n.start) { 
        startExists = true;  
        startX = j; 
        startY = i;
      } 

      if (n.goal) { 
        endExists = true;
        endX = j;
        endY = i;
      }
    }
  }

  if (!startExists || !endExists) return;

  boolean change = true;
  int index = 0;
  while (change) {
    change = false;
    map[endY][endX].value = 0;
    index++;
    for (int i = 0; i < 10; i++) {
      for (int j = 0; j < 10; j++) {
        Node c = map[j][i];
        if (c.value != -1) continue;
        if (c.wall) continue;

        // North node
        if (j != 0) {
          Node n = map[j-1][i];
          if (n.value != -1 && n.value < index) {
            c.value = index;
            change = true;
            continue;
          }
        }

        // West node
        if (i != 0) {
          Node w = map[j][i-1];
          if (w.value != -1 && w.value < index) {
            c.value = index;
            change = true;
            continue;
          }
        }

        // East node
        if (j != 9) {
          Node e = map[j+1][i];
          if (e.value != -1 && e.value < index) {
            c.value = index;
            change = true;
            continue;
          }
        }

        // South node
        if (i != 9) {
          Node s = map[j][i+1];
          if (s.value != -1 && s.value < index) {
            c.value = index;
            change = true;
            continue;
          }
        }
      }
    }
  }

  // Build the path
  recursionCount = 0;
  buildPath(startX, startY);
}
static int recursionCount = 0;
void buildPath(int startX, int startY) {
  recursionCount++;
  if(recursionCount > 100) return;
  Node c = map[startY][startX];
  c.path = true;
  if (c.value == 0) return;
  int min = 9000;
  Direction direction = Direction.SOUTH;

  // Check west
  if (startX != 0) {
    Node w = map[startY][startX-1];
    if (w.value < min && w.value != -1) {
      min = w.value;
      direction = Direction.WEST;
    }
  }

  // Check east
  if (startX != 9) {
    Node e = map[startY][startX+1];
    if (e.value < min && e.value != -1) { 
      min = e.value;
      direction = Direction.EAST;
    }
  }

  // Check north
  if (startY != 0) {
    Node n = map[startY-1][startX];
    if (n.value < min && n.value != -1) {
      min = n.value;
      direction = Direction.NORTH;
    }
  }

  // Check south
  if (startY != 9) {
    Node s = map[startY+1][startX];
    if (s.value < min && s.value != -1) {
      min = s.value;
      direction = Direction.SOUTH;
    }
  }

  switch(direction) {
  case NORTH: 
    buildPath(startX, startY - 1); 
    break;
  case SOUTH: 
    buildPath(startX, startY + 1); 
    break;
  case EAST: 
    buildPath(startX + 1, startY); 
    break;
  case WEST: 
    buildPath(startX - 1, startY); 
    break;
  }
}
