# Bluetooth-Controlled Maze-Solving Robot Car

An ESP32-based robot car that explores a physical maze using ultrasonic sensors, maps out the walls it discovers, then computes and drives the **shortest path** to the goal — controllable manually over Bluetooth or run autonomously in maze-solving mode.

## How It Works

The robot solves the maze in three phases:

1. **Exploration (DFS)** — Since the robot has no prior knowledge of the maze and can only move to physically adjacent cells, it uses **Depth-First Search** to explore: driving into unvisited open cells, and backtracking (reversing + turning) whenever it hits a dead end. As it explores, it records which directions are open or blocked at every cell using its three ultrasonic sensors (front, left, right).

2. **Shortest-Path Computation (BFS)** — Once exploration is complete, the robot has a full internal map of the maze. It then runs **Breadth-First Search** on that map — a pure computation step, no motor movement — to find the guaranteed shortest route from start to goal. BFS is used here specifically because, on an unweighted grid, it explores the map one "ring" of distance at a time, so the first time it reaches the goal, that path is mathematically guaranteed to be the shortest possible one.

3. **Execution** — The robot drives the BFS-computed shortest path to the goal.

> **Why DFS *and* BFS, not just one?** DFS's only job is *discovering* the maze layout, since a real robot has to physically visit cells to know where the walls are — it can't skip ahead. BFS's job is *choosing the best route* once the layout is known. The final path the robot drives is always the BFS result; DFS never decides the route itself.

## Hardware

| Component | Details |
|---|---|
| Microcontroller | ESP32 (Bluetooth Serial for wireless control) |
| Motor Driver | L298N |
| Motors | 2x DC geared motors (differential drive) |
| Sensors | 3x HC-SR04 ultrasonic sensors (front, left, right) |
| Chassis | 2WD robot car chassis with caster wheel |

### Pin Configuration

| Signal | Pin |
|---|---|
| ENA (Left motor PWM) | 5 |
| IN1 / IN2 (Left motor direction) | 18 / 19 |
| ENB (Right motor PWM) | 21 |
| IN3 / IN4 (Right motor direction) | 22 / 23 |
| Front ultrasonic — Trig / Echo | 13 / 12 |
| Left ultrasonic — Trig / Echo | 27 / 14 |
| Right ultrasonic — Trig / Echo | 26 / 25 |

## Bluetooth Commands

Connect to the ESP32 over Bluetooth (device name: `PK_ROBO`) using any serial Bluetooth terminal app, then send:

| Command | Action |
|---|---|
| `F` | Move forward one cell |
| `B` | Move backward one cell |
| `L` | Turn left 90° |
| `R` | Turn right 90° |
| `S` | Stop |
| `+` | Increase motor speed |
| `-` | Decrease motor speed |
| `M` | Run full maze solve: explore (DFS) → compute shortest path (BFS) → drive it |

## Calibration

Two things must be measured and set for your specific build before running a maze solve:

1. **`CELL_SIZE_CM`** — the real-world size of one maze cell. Measure your physical maze and update this constant.
2. **`TIME_PER_REV_MS`** — how long one full wheel rotation takes at the configured `motorSpeed`. There's no wheel encoder on this robot, so this must be measured by hand:
   - Mark one wheel with tape
   - Send `F` to run the motor
   - Time exactly one full rotation with a stopwatch
   - Update `TIME_PER_REV_MS` with your measured value (in ms)

From these two values plus the measured **wheel circumference (23.5 cm)**, the code automatically computes how long the robot needs to drive to cover exactly one maze cell — no manual delay-tuning required beyond this calibration step.

You'll also want to set:
- `MAZE_ROWS` / `MAZE_COLS` — your maze's grid dimensions
- `START_X` / `START_Y` and `GOAL_X` / `GOAL_Y` — start and goal cells
- `WALL_THRESHOLD_CM` — the ultrasonic distance below which a wall is considered present

## Setup

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) with ESP32 board support.
2. Open `robot_car_maze_solver.ino`.
3. Update the calibration constants above for your build.
4. Select your ESP32 board and port, then upload.
5. Pair with `PK_ROBO` over Bluetooth and send `M` to run a maze solve.
