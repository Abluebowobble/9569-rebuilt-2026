# Middle Depot Auton - Changelog

**Branch:** `feature/middle-auton`
**Date:** 2026-04-06
**Task:** Create a middle autonomous that drives to the depot and collects balls (assigned by Isabella)

---

## Files Changed

### 1. `src/main/java/frc/robot/Constants.java`

**What changed:** Added 4 new waypoint constants for the middle auton in `WaypointConstants` (lines 82-85).

```java
// Line 82
public static final Translation2d BLUE_MIDDLE_START = new Translation2d(3.540, 3.989);
// Line 83
public static final Translation2d BLUE_MIDDLE_PREPARE_DEPOT = new Translation2d(1.52, 5.963);
// Line 84
public static final Translation2d BLUE_MIDDLE_DEPOT = new Translation2d(0.706, 5.963);
// Line 85
public static final Translation2d BLUE_MIDDLE_SHOOT = new Translation2d(3.303, 3.97);
```

| Constant | Coordinates | Purpose |
|---|---|---|
| `BLUE_MIDDLE_START` | (3.540, 3.989) | Robot starting position at middle |
| `BLUE_MIDDLE_PREPARE_DEPOT` | (1.52, 5.963) | Intermediate point before depot (safe distance from wall) |
| `BLUE_MIDDLE_DEPOT` | (0.706, 5.963) | Depot intake position (matches existing BLUE_1_DEPOT_INTAKE X) |
| `BLUE_MIDDLE_SHOOT` | (3.303, 3.97) | Scoring position after collecting balls |

---

### 2. `src/main/java/frc/robot/Commands/Autons.java`

**What changed:** Added `middleDepotAuton()` method (lines 511-573).

**Method signature:** `public static Command middleDepotAuton(GeneralRobotCommands generalRobotCommands)`

**Auton sequence (6 steps):**

| Step | Line(s) | Command | Method Used | Why This Method |
|---|---|---|---|---|
| 1 | 527-530 | Reset odometry at middle start | `resetOdometry()` | Initialize pose |
| 2 | 533-539 | Cruise to prepare depot point | `driveToWayPointWithAngle` | Smooth constant velocity, no PID needed in open field |
| 3 | 542-550 | Drive into depot + intake simultaneously | `ParallelDeadlineGroup` wrapping `driveToWayPointWithAngle` + `intakeCommand` | Waypoint drive avoids PID oscillation near wall; 1/3 speed for safe approach; intake runs in parallel |
| 4 | 553-556 | Leave depot back to prepare point | `driveToWithAngle` | PID is safe in open field, need precise positioning |
| 5 | 559-564 | Return to scoring position + spin up shooter | `ParallelDeadlineGroup` wrapping `driveToWithAngle` + shooter `runCommand` | PID for precise scoring alignment; shooter spins up during transit |
| 6 | 567 | Shoot when ready | `shootWhenReady()` | Waits for shooter velocity, then feeds balls |

**Design decisions:**
- Used `driveToWayPointWithAngle` (constant velocity) for depot approach to prevent PID oscillation near the field wall
- Used `MAX_SPEED.div(3)` for depot intake approach (slow, controlled entry)
- Used `driveToWithAngle` (PID) for return-to-score because precision matters and it's in open field
- Follows the exact depot collection pattern from the commented-out code in `testleftAuton` (lines 188-216) and `testRightAuton` (lines 335-363)
- `allianceRelative()` handles blue/red flipping automatically

---

### 3. `src/main/java/frc/robot/RobotContainer.java`

**What changed:** Registered the new auton in the auto chooser (line 123).

```java
// Line 123
autoChooser.addOption("Middle Depot", Autons.middleDepotAuton(generalRobotCommands));
```

Selectable as **"Middle Depot"** on the SmartDashboard auto chooser.

> **Note:** `getAutonomousCommand()` (line 321) currently hardcodes `return shootAuton();` instead of `return autoChooser.getSelected();`. You will need to uncomment line 325 and remove line 326 to actually use the chooser at competition.

---

## How to Test

1. Open the project in VS Code / WPILib
2. Press `Ctrl+Shift+P` -> type "Simulate" -> select "Simulate Robot Code"
3. Open AdvantageScope and connect to the simulated robot
4. Select **"Middle Depot"** from the Auto Chooser on SmartDashboard
5. Enable autonomous mode and observe the path

## Coordinates to Tune

If the path doesn't look right in simulation, these are the values to adjust in `Constants.java > WaypointConstants`:
- `BLUE_MIDDLE_PREPARE_DEPOT` — move X higher (e.g. 2.0) if robot cuts too close to wall on approach
- `BLUE_MIDDLE_DEPOT` — move X higher (e.g. 0.8+) if robot gets too close to wall
- `BLUE_MIDDLE_SHOOT` — adjust to match your team's preferred scoring spot from middle
