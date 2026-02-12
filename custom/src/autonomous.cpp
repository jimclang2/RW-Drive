#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/src/user.cpp
// Format: returnType functionName() { code }

/*
 * tuningAuton
 * Use this to tune your PID values.
 * Start with driveTo to tune distance PID, then turnToAngle for turn PID.
 */
void tuningAuton() {
  driveTo(24, 3000);
  turnToAngle(90, 2000);
  turnToAngle(0, 2000);
  driveTo(-24, 3000);
}

/*
 * skills_auton
 * Autonomous skills routine.
 * TODO: Rewrite using RW-Template motion functions (moveToPoint, driveTo, turnToAngle, boomerang)
 * Your old LemLib routes won't work here — the motion functions have different parameters.
 * 
 * RW-Template motion function reference:
 *   driveTo(distance_inches, time_limit_ms, exit, max_output)
 *   turnToAngle(angle_deg, time_limit_ms, exit, max_output)
 *   moveToPoint(x, y, dir, time_limit_ms, exit, max_output, overturn)
 *   boomerang(x, y, dir, angle, dlead, time_limit_ms, exit, max_output, overturn)
 *   curveCircle(result_angle_deg, center_radius, time_limit_ms, exit, max_output)
 *   swing(swing_angle, drive_direction, time_limit_ms, exit, max_output)
 *
 * Pneumatics:
 *   descore_piston.set(true/false)
 *   unloader_piston.set(true/false)
 *   midscoring_piston.set(true/false)
 *
 * Motors:
 *   intake_motor.spin(fwd/reverse, voltage, volt)
 *   outtake_motor.spin(fwd/reverse, voltage, volt)
 *   intake_motor.stop(hold/brake/coast)
 */
void skills_auton() {
  // TODO: Write your skills autonomous here
}

/*
 * leftAuton
 * Left-side match autonomous.
 * TODO: Write using RW-Template motion functions
 */
void leftAuton() {
  // TODO: Write your left autonomous here
}

/*
 * rightAuton
 * Right-side match autonomous.
 * TODO: Write using RW-Template motion functions
 */
void rightAuton() {
  // TODO: Write your right autonomous here
}

/*
 * rightAutonDescore
 * Right-side match autonomous with descore.
 * TODO: Write using RW-Template motion functions
 */
void rightAutonDescore() {
  // TODO: Write your right autonomous descore here
}