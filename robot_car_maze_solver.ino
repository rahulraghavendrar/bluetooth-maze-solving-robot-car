#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// ======================================================
// ---------------- Pin Configuration ------------------
// ======================================================
// Motor Driver (L298N)
const int ENA = 5;    // Enable pin for Left Motor (PWM)
const int IN1 = 18;   // Direction pin 1 for Left Motor
const int IN2 = 19;   // Direction pin 2 for Left Motor
const int ENB = 21;   // Enable pin for Right Motor (PWM)
const int IN3 = 22;   // Direction pin 1 for Right Motor
const int IN4 = 23;   // Direction pin 2 for Right Motor

// Ultrasonic Sensors (HC-SR04 style: Trig + Echo per sensor)
const int TRIG_FRONT = 13, ECHO_FRONT = 12;
const int TRIG_LEFT  = 27, ECHO_LEFT  = 14;
const int TRIG_RIGHT = 26, ECHO_RIGHT = 25;

// ======================================================
// ---------------- Tunable Parameters ------------------
// ======================================================
float motorSpeed = 175;        // 0-255
float leftMotorCal  = 1.0;     // adjust if one side runs faster/slower
float rightMotorCal = 1.0;

// ---- Maze physical dimensions ----
// CELL_SIZE_CM is the real-world size of one maze cell (measure your
// physical maze and set this accordingly).
const float CELL_SIZE_CM = 30.0;   // e.g. 30cm x 30cm per maze cell

// ---- Wheel calibration ----
// WHEEL_CIRCUMFERENCE_CM = distance the robot travels for ONE full
// rotation of the back wheel (measured: 23.5 cm).
const float WHEEL_CIRCUMFERENCE_CM = 23.5;

// TIME_PER_REV_MS = how long (in ms) ONE full wheel rotation takes at
// the motorSpeed set below. There's no wheel encoder on this robot, so
// this MUST be measured by hand: mark one wheel with tape, run the
// motor at 'motorSpeed' via the 'F' command, and time (stopwatch) how
// long it takes to complete exactly one full rotation. Update this
// value with your measured result -- moveDelay is then computed
// automatically from it, so you never have to hand-tune moveDelay again.
const float TIME_PER_REV_MS = 300.0;  // <-- PLACEHOLDER: replace with your measured value

int moveDelay;                     // computed in setup() from the above
int turnDelay = 600;               // ms for a 90-degree turn (still hand-tuned;
                                    // rotation-in-place isn't a straight-line
                                    // wheel-circumference calculation)

const float WALL_THRESHOLD_CM = 12.0;  // distance below this = wall present

// ---- Maze grid dimensions ----
const int MAZE_ROWS = 8;
const int MAZE_COLS = 8;

// Start and goal cells (grid coordinates, 0-indexed)
const int START_X = 0, START_Y = 0;
const int GOAL_X = 7, GOAL_Y = 7;

// ======================================================
// ---------------- Maze / Search State -------------------
// ======================================================
enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
int currentDir = NORTH;
int curX = START_X, curY = START_Y;

bool visited[MAZE_ROWS][MAZE_COLS];

// wallMap[x][y][d] = true if a wall blocks movement from (x,y) in
// direction d. Filled in during DFS exploration via sensor readings.
bool wallMap[MAZE_ROWS][MAZE_COLS][4];

const int dx[4] = { 0, 1, 0, -1 };   // N, E, S, W
const int dy[4] = { 1, 0, -1, 0 };

// ======================================================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("PK_ROBO");

  // Compute how long the robot must drive to cover one maze cell:
  // rotations needed = CELL_SIZE_CM / WHEEL_CIRCUMFERENCE_CM
  // moveDelay(ms)     = rotations needed * TIME_PER_REV_MS
  float rotationsPerCell = CELL_SIZE_CM / WHEEL_CIRCUMFERENCE_CM;
  moveDelay = (int)(rotationsPerCell * TIME_PER_REV_MS);

  Serial.println("Ready. Send 'M' to explore (DFS) then solve (BFS) the maze.");
  Serial.print("Cell size: "); Serial.print(CELL_SIZE_CM); Serial.println(" cm");
  Serial.print("Wheel circumference: "); Serial.print(WHEEL_CIRCUMFERENCE_CM); Serial.println(" cm/rev");
  Serial.print("Rotations per cell: "); Serial.println(rotationsPerCell);
  Serial.print("Computed moveDelay: "); Serial.print(moveDelay); Serial.println(" ms");

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  memset(visited, false, sizeof(visited));
  memset(wallMap, false, sizeof(wallMap));
  stopMotors();
}

// ======================================================
void loop() {
  if (SerialBT.available()) {
    char command = toupper(SerialBT.read());
    Serial.print("Command: "); Serial.println(command);

    switch (command) {
      case 'F': moveForward();  delay(moveDelay); stopMotors(); break;
      case 'B': moveBackward(); delay(moveDelay); stopMotors(); break;
      case 'L': turnLeft90();   break;
      case 'R': turnRight90();  break;
      case 'S': stopMotors();   break;
      case '+': increaseSpeed(); break;
      case '-': decreaseSpeed(); break;
      case 'M': runMazeSolve(); break;
      default:  Serial.println("Unknown command");
    }
  }
}

void runMazeSolve() {
  Serial.println("Phase 1: Exploring maze (DFS)...");
  visited[curX][curY] = true;
  exploreMazeDFS(curX, curY);
  faceDirection(NORTH);
  Serial.println("Exploration complete.");

  Serial.println("Phase 2: Computing shortest path (BFS)...");
  int path[MAZE_ROWS * MAZE_COLS][2];
  int pathLen = solveMazeBFS(path);

  if (pathLen == -1) {
    Serial.println("No path found to goal.");
    return;
  }

  Serial.print("Shortest path found, "); Serial.print(pathLen);
  Serial.println(" cells. Phase 3: Driving path...");
  drivePath(path, pathLen);
  Serial.println("Goal reached.");
}

// ======================================================
// ---------------- Phase 1: DFS Exploration ---------------
// ======================================================
// Physically drives the robot through every reachable cell, recording
// which directions are open/blocked into wallMap as it goes.
void exploreMazeDFS(int x, int y) {
  int absFront = currentDir;
  int absLeft  = (currentDir + 3) % 4;
  int absRight = (currentDir + 1) % 4;

  bool wallFront = isWallPresent(readDistanceCM(TRIG_FRONT, ECHO_FRONT));
  bool wallLeft  = isWallPresent(readDistanceCM(TRIG_LEFT,  ECHO_LEFT));
  bool wallRight = isWallPresent(readDistanceCM(TRIG_RIGHT, ECHO_RIGHT));

  wallMap[x][y][absFront] = wallFront;
  wallMap[x][y][absLeft]  = wallLeft;
  wallMap[x][y][absRight] = wallRight;

  tryDirection(absFront, wallFront, x, y);
  tryDirection(absLeft,  wallLeft,  x, y);
  tryDirection(absRight, wallRight, x, y);
}

void tryDirection(int dir, bool wallPresent, int x, int y) {
  if (wallPresent) return;

  int nx = x + dx[dir];
  int ny = y + dy[dir];
  if (nx < 0 || ny < 0 || nx >= MAZE_ROWS || ny >= MAZE_COLS) return;
  if (visited[nx][ny]) return;

  faceDirection(dir);
  moveForward(); delay(moveDelay); stopMotors();

  curX = nx; curY = ny;
  visited[nx][ny] = true;
  wallMap[nx][ny][(dir + 2) % 4] = false;  // reverse door is open too

  exploreMazeDFS(nx, ny);

  // Backtrack to (x, y)
  faceDirection((dir + 2) % 4);
  moveForward(); delay(moveDelay); stopMotors();
  faceDirection(dir);

  curX = x; curY = y;
}

// ======================================================
// ---------------- Phase 2: BFS Shortest Path -------------
// ======================================================
// Runs BFS over the wallMap discovered during exploration. Unlike DFS,
// BFS expands the search one "ring" of distance at a time, so the
// first time it reaches the goal, that path is guaranteed to be the
// shortest possible one on the discovered map.
// Returns path length, fills 'path' with the sequence of (x,y) cells
// from start to goal (inclusive). Returns -1 if no path exists.
int solveMazeBFS(int path[][2]) {
  bool bfsVisited[MAZE_ROWS][MAZE_COLS];
  int parentX[MAZE_ROWS][MAZE_COLS];
  int parentY[MAZE_ROWS][MAZE_COLS];
  memset(bfsVisited, false, sizeof(bfsVisited));

  int queueX[MAZE_ROWS * MAZE_COLS];
  int queueY[MAZE_ROWS * MAZE_COLS];
  int qHead = 0, qTail = 0;

  queueX[qTail] = START_X; queueY[qTail] = START_Y; qTail++;
  bfsVisited[START_X][START_Y] = true;
  parentX[START_X][START_Y] = -1;
  parentY[START_X][START_Y] = -1;

  bool found = false;
  while (qHead < qTail) {
    int x = queueX[qHead], y = queueY[qHead]; qHead++;

    if (x == GOAL_X && y == GOAL_Y) { found = true; break; }

    for (int d = 0; d < 4; d++) {
      if (wallMap[x][y][d]) continue;  // wall blocks this direction

      int nx = x + dx[d], ny = y + dy[d];
      if (nx < 0 || ny < 0 || nx >= MAZE_ROWS || ny >= MAZE_COLS) continue;
      if (bfsVisited[nx][ny]) continue;

      bfsVisited[nx][ny] = true;
      parentX[nx][ny] = x;
      parentY[nx][ny] = y;
      queueX[qTail] = nx; queueY[qTail] = ny; qTail++;
    }
  }

  if (!found) return -1;

  // Reconstruct path by walking parents backward from goal to start
  int tempPath[MAZE_ROWS * MAZE_COLS][2];
  int len = 0;
  int x = GOAL_X, y = GOAL_Y;
  while (x != -1) {
    tempPath[len][0] = x; tempPath[len][1] = y; len++;
    int px = parentX[x][y], py = parentY[x][y];
    x = px; y = py;
  }

  // Reverse into 'path' so it goes start -> goal
  for (int i = 0; i < len; i++) {
    path[i][0] = tempPath[len - 1 - i][0];
    path[i][1] = tempPath[len - 1 - i][1];
  }
  return len;
}

// ======================================================
// ---------------- Phase 3: Drive the Path ----------------
// ======================================================
void drivePath(int path[][2], int len) {
  faceDirection(NORTH);
  curX = START_X; curY = START_Y;

  for (int i = 1; i < len; i++) {
    int nx = path[i][0], ny = path[i][1];
    int stepDx = nx - curX, stepDy = ny - curY;

    int dir = -1;
    for (int d = 0; d < 4; d++) {
      if (dx[d] == stepDx && dy[d] == stepDy) { dir = d; break; }
    }
    if (dir == -1) continue;  // shouldn't happen on a valid path

    faceDirection(dir);
    moveForward(); delay(moveDelay); stopMotors();
    curX = nx; curY = ny;
  }
}

// Rotates the robot in place until it faces 'targetDir'
void faceDirection(int targetDir) {
  int diff = (targetDir - currentDir + 4) % 4;
  if (diff == 1) {
    turnRight90();
  } else if (diff == 3) {
    turnLeft90();
  } else if (diff == 2) {
    turnRight90(); turnRight90();
  }
  currentDir = targetDir;
}

// ======================================================
// ---------------- Ultrasonic Sensing --------------------
// ======================================================
float readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // 30ms timeout
  if (duration == 0) return 999.0;                // no echo = treat as open

  return duration * 0.0343 / 2.0;  // speed of sound -> cm
}

bool isWallPresent(float distanceCM) {
  return distanceCM < WALL_THRESHOLD_CM;
}

// ======================================================
// ---------------- Motor Control Functions ---------------
// ======================================================
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed * leftMotorCal);
  analogWrite(ENB, motorSpeed * rightMotorCal);
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed * leftMotorCal);
  analogWrite(ENB, motorSpeed * rightMotorCal);
}

void turnLeft90() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed * leftMotorCal);
  analogWrite(ENB, motorSpeed * rightMotorCal);
  delay(turnDelay);
  stopMotors();
}

void turnRight90() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed * leftMotorCal);
  analogWrite(ENB, motorSpeed * rightMotorCal);
  delay(turnDelay);
  stopMotors();
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ======================================================
// ---------------- Speed Control -------------------------
// ======================================================
void increaseSpeed() {
  motorSpeed += 25;
  motorSpeed = constrain(motorSpeed, 50, 255);
  Serial.print("Speed increased to: "); Serial.println(motorSpeed);
}

void decreaseSpeed() {
  motorSpeed -= 25;
  motorSpeed = constrain(motorSpeed, 50, 255);
  Serial.print("Speed decreased to: "); Serial.println(motorSpeed);
}
