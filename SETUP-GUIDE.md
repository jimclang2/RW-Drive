# RW-Template Setup Guide — Next Steps

Your robot hardware is fully configured. Here's what to do from here.

---

## Step 1: Build & Upload

1. Open this project in VS Code with the **VEX Robotics Extension** installed
2. Click the **Build** button in the VEX toolbar (or `Ctrl+Shift+B`)
3. Fix any build errors if they come up
4. Connect your V5 Brain via USB and click **Upload**

---

## Step 2: Verify Motor Directions

Before anything else, make sure all motors spin the right way:

1. Run the robot in **driver control** mode
2. Push both joysticks forward — the robot should drive **forward**
3. If a side drives the wrong way, flip the `true`/`false` (reversed) flag for that motor in `custom/src/robot-config.cpp`

---

## Step 3: Tune PID

PID values in `custom/src/robot-config.cpp` are marked with `// CHANGE: Tune these!`

### Distance PID (straight driving)

1. Set `auton_selected = 1` in `custom/src/user.cpp` to run `tuningAuton()`
2. Start with just `distance_kp` — increase until the robot overshoots, then back off
3. Add `distance_kd` to dampen oscillation
4. Add small `distance_ki` only if the robot consistently falls short

### Turn PID (turning in place)

1. Same process — tune `turn_kp` first, then `turn_kd`
2. Test multiple target angles (90°, 180°, 45°) to ensure consistency

### Heading Correction PID

- This keeps the robot driving straight — tune after distance and turn PID are solid
- Usually needs less aggressive values than the turn PID

---

## Step 4: Verify Subsystems

Test each in **driver control** mode:

| Button | Action                                            |
| ------ | ------------------------------------------------- |
| **R2** | Toggle intake forward                             |
| **R1** | Toggle intake reverse                             |
| **L1** | Toggle outtake forward                            |
| **X**  | Toggle mid-scoring mode (unjam + reverse outtake) |
| **A**  | Hold to retract Descore (releases on let go)      |
| **L2** | Toggle Unloader                                   |

---

## Step 5: Write Autonomous Routines

Edit `custom/src/autonomous.cpp` — placeholder functions are ready for you:

- `skills_auton()`
- `leftAuton()`
- `rightAuton()`
- `rightAutonDescore()`

### Available Motion Functions

```cpp
// Drive straight (inches)
driveTo(distance, time_limit_ms, exit, max_output);

// Turn to absolute heading (degrees)
turnToAngle(angle, time_limit_ms, exit, max_output);

// Drive to a field coordinate
moveToPoint(x, y, dir, time_limit_ms, exit, max_output, overturn);
// dir: 1 = forward, -1 = backward

// Smooth curved path to a pose (position + heading)
boomerang(x, y, dir, angle, dlead, time_limit_ms, exit, max_output, overturn);

// Circular arc path
curveCircle(result_angle_deg, center_radius, time_limit_ms, exit, max_output);

// Swing turn (one side stationary)
swing(swing_angle, drive_direction, time_limit_ms, exit, max_output);
```

### Motor & Pneumatic Controls in Auton

```cpp
// Intake / Outtake
intake_motor.spin(fwd, 12, volt);     // Full speed forward
intake_motor.spin(reverse, 12, volt); // Full speed reverse
intake_motor.stop(hold);              // Stop and hold

outtake_motor.spin(fwd, 12, volt);
outtake_motor.stop(hold);

// Pneumatics
descore_piston.set(true);    // or false
unloader_piston.set(true);   // or false
midscoring_piston.set(true); // or false

// Direct chassis control
driveChassis(left_volts, right_volts); // e.g. driveChassis(6, 6)
stopChassis(hold);                     // or brake, coast

// Delays
wait(500, msec);

// Set/reset position
x_pos = 0; y_pos = 0;  // Reset odometry position
correct_angle = 0;      // Reset heading reference
```

### Key Differences from LemLib

| LemLib                                  | RW-Template                                                   |
| --------------------------------------- | ------------------------------------------------------------- |
| `chassis.moveToPoint(x, y, timeout)`    | `moveToPoint(x, y, dir, timeout)` — requires direction (1/-1) |
| `chassis.turnToHeading(angle, timeout)` | `turnToAngle(angle, timeout)`                                 |
| `chassis.setPose(x, y, heading)`        | Set `x_pos`, `y_pos`, `correct_angle` directly                |
| `{.forwards = false}`                   | Use `dir = -1` parameter                                      |
| `{.maxSpeed = 80}`                      | Use `max_output` parameter (in volts, max 12)                 |
| `chassis.moveToPose(...)`               | `boomerang(x, y, dir, angle, dlead, timeout)`                 |
| `pros::delay(ms)`                       | `wait(ms, msec)`                                              |

Select your auton in `custom/src/user.cpp` by changing `auton_selected` in `runAutonomous()`.

---

## Step 6: Distance Sensors (Future)

When you're ready to add distance sensors:

1. Set the correct ports in `custom/src/robot-config.cpp` (lines 43-46, currently on placeholder ports 1-4)
2. Set the sensor offsets (lines 101-104) — distance from each sensor to the robot's center (in inches)
3. Call `resetPositionFront()`, `resetPositionBack()`, `resetPositionLeft()`, or `resetPositionRight()` in your autonomous routines

---

## File Reference

Only edit files in the `custom/` folder — core files will be overwritten on template updates.

| File                            | Purpose                                         |
| ------------------------------- | ----------------------------------------------- |
| `custom/src/robot-config.cpp`   | Motor ports, sensors, PID values, measurements  |
| `custom/include/robot-config.h` | Device declarations (must match .cpp)           |
| `custom/src/user.cpp`           | Driver control, pre-auton init, auton selection |
| `custom/src/autonomous.cpp`     | Your autonomous routines                        |
| `custom/include/autonomous.h`   | Autonomous function declarations                |
