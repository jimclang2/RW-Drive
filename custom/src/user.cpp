#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include "../custom/include/robot-config.h"
#include <cmath>

// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  int auton_selected = 1;
  switch(auton_selected) {
    case 1:
      tuningAuton();
      break;
    case 2:
      skills_auton();
      break;  
    case 3:
      leftAuton();
      break;
    case 4:
      rightAuton();
      break;
    case 5:
      rightAutonDescore();
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
  }
}

// ============================================================================
// EXPO DRIVE CURVE (Ported from LemLib)
// Matches: lemlib::ExpoDriveCurve(3, 10, 1.05)
// Parameters: deadband=3, minOutput=10, curve=1.05
// ============================================================================
double expoCurve(double input, double deadband = 3, double minOutput = 10, double curve = 1.05) {
  // If input is within deadband, return 0
  if (fabs(input) <= deadband) return 0;
  
  // LemLib expo curve formula
  double output = (exp(-deadband / 10.0) + exp((fabs(input) - 127.0) / 10.0) * (1 - exp(-deadband / 10.0))) * input;
  
  // Apply curve factor
  output *= curve;
  
  // Apply minimum output (ensure motors actually move when joystick is past deadband)
  if (fabs(output) < minOutput && fabs(input) > deadband) {
    output = minOutput * (input > 0 ? 1 : -1);
  }
  
  return output;
}

// ============================================================================
// SUBSYSTEM STATE VARIABLES
// ============================================================================

// Intake toggle state
bool intake_toggle_forward = false;
bool intake_toggle_reverse = false;
bool r1_last = false;
bool a_last = false;

// Outtake toggle state
bool l1_combo_active = false;
bool l1_last = false;
bool midscoring_mode = false;
bool x_last = false;
bool is_unjamming = false;
double unjam_start_time = 0;

// Pneumatic state
bool unloader_state = false;
bool l2_last = false;

// ============================================================================
// SUBSYSTEM UPDATE FUNCTIONS
// ============================================================================

/*
 * updateIntake
 * Toggle-based intake control using R1 (forward) and A (reverse).
 * Pressing R1 toggles forward spin; pressing A toggles reverse spin.
 * R1/A will cancel L1 combo mode if active.
 * Intake is blocked during mid-scoring or L1 combo (controlled elsewhere).
 */
void updateIntake() {
  bool r1_current = controller_1.ButtonR1.pressing();
  bool a_current = controller_1.ButtonA.pressing();

  // Detect rising edge for toggles (ignored during mid-scoring)
  if (!midscoring_mode) {
    if (r1_current && !r1_last) {
      // Cancel L1 combo if active (R1 takes priority)
      if (l1_combo_active) {
        l1_combo_active = false;
        outtake_motor.stop(hold);
      }
      intake_toggle_forward = !intake_toggle_forward;
      if (intake_toggle_forward) intake_toggle_reverse = false;
    }
    if (a_current && !a_last) {
      // Cancel L1 combo if active (A takes priority)
      if (l1_combo_active) {
        l1_combo_active = false;
        outtake_motor.stop(hold);
      }
      intake_toggle_reverse = !intake_toggle_reverse;
      if (intake_toggle_reverse) intake_toggle_forward = false;
    }
  }

  r1_last = r1_current;
  a_last = a_current;

  // Only control intake here if NOT blocked by mid-scoring or L1 combo
  if (!midscoring_mode && !l1_combo_active) {
    if (intake_toggle_forward) {
      intake_motor.spin(fwd, 12, volt);
    } else if (intake_toggle_reverse) {
      intake_motor.spin(reverse, 12, volt);
    } else {
      intake_motor.stop(hold);
    }
  }
}

/*
 * updateOuttake
 * L1 toggles combo mode (intake forward + outtake reverse). X toggles mid-scoring.
 * L1 combo overrides R1/A intake; R1/A cancel L1 combo. Mid-scoring blocks L1.
 * Mid-scoring mode: retracts piston, runs unjam sequence (brief reverse), 
 * then runs intake forward + outtake reverse for scoring.
 */
void updateOuttake() {
  // Handle unjam sequence timing
  if (is_unjamming) {
    double elapsed = Brain.Timer.value() * 1000.0 - unjam_start_time;
    if (elapsed >= 100) { // 100ms unjam duration
      is_unjamming = false;
    } else {
      intake_motor.spin(fwd, 12, volt);    // Unjam: run intake forward
      outtake_motor.spin(reverse, 12, volt); // Unjam: run outtake reverse
      return;
    }
  }

  // Mid-scoring toggle (Button X)
  bool x_current = controller_1.ButtonX.pressing();
  if (x_current && !x_last) {
    midscoring_mode = !midscoring_mode;

    if (midscoring_mode) {
      // ENTERING mid-scoring mode
      l1_combo_active = false;
      midscoring_piston.set(true);  // Retract piston
      is_unjamming = true;
      unjam_start_time = Brain.Timer.value() * 1000.0;
    } else {
      // EXITING mid-scoring mode
      midscoring_piston.set(false);  // Extend piston
      is_unjamming = false;
      intake_motor.stop(hold);
      outtake_motor.stop(hold);
      l1_combo_active = false;
      x_last = x_current;
      return;
    }
  }
  x_last = x_current;

  double outtake_voltage = 0;

  if (midscoring_mode && !is_unjamming) {
    // Mid-scoring: intake forward, outtake reverse at reduced speed
    intake_motor.spin(reverse, 12, volt);
    outtake_voltage = -12; // Full reverse speed
    // Update L1 state to prevent "stored" presses
    l1_last = controller_1.ButtonL1.pressing();
  } else if (!midscoring_mode) {
    // Normal mode: L1 toggles combo (intake forward + outtake reverse)
    bool l1_current = controller_1.ButtonL1.pressing();
    if (l1_current && !l1_last) {
      l1_combo_active = !l1_combo_active;
      if (l1_combo_active) {
        // Clear intake toggles so they don't resume after combo ends
        intake_toggle_forward = false;
        intake_toggle_reverse = false;
      }
    }
    l1_last = l1_current;

    if (l1_combo_active) {
      intake_motor.spin(fwd, 12, volt);   // Intake forward
      outtake_voltage = -12;               // Outtake reverse (full speed)
    }
  }

  if (outtake_voltage != 0) {
    outtake_motor.spin(fwd, outtake_voltage, volt);
  } else if (!midscoring_mode && !is_unjamming) {
    outtake_motor.stop(hold);
  }
}

/*
 * updatePneumatics
 * R2 button: hold to retract Descore, release to extend (starts UP/extended)
 * L2 button: toggle Unloader on/off
 */
void updatePneumatics() {
  // Descore (Button R2) — hold to retract, release to extend
  bool r2_current = controller_1.ButtonR2.pressing();
  descore_piston.set(!r2_current);

  // Unloader (Button L2) — toggle
  bool l2_current = controller_1.ButtonL2.pressing();
  if (l2_current && !l2_last) {
    unloader_state = !unloader_state;
    unloader_piston.set(unloader_state);
  }
  l2_last = l2_current;
}

// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;

void runDriver() {
  stopChassis(coast);
  heading_correction = false;
  
  // Descore starts UP immediately
  descore_piston.set(true);
  
  while (true) {
    // Read joystick values [-100, 100]
    ch3 = controller_1.Axis3.value(); // Left stick Y
    ch2 = controller_1.Axis2.value(); // Right stick Y

    // Apply expo curve (matches LemLib ExpoDriveCurve(3, 10, 1.05))
    // Then scale to voltage range for driveChassis (which takes volts)
    double left_drive = expoCurve(ch3) * 0.12;  // Scale [-127,127] range to ~[-12,12] volts
    double right_drive = expoCurve(ch2) * 0.12;
    
    driveChassis(left_drive, right_drive);

    // Update subsystems
    updateOuttake();  // Must run before intake (controls mid-scoring blocking)
    updateIntake();
    updatePneumatics();

    wait(20, msec); 
  }
}

void runPreAutonomous() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }

  double current_heading = inertial_sensor.heading();
  Brain.Screen.print(current_heading);

  // Set brake modes
  left_chassis.setStopping(brake);    // Prevents drifting, smooth control
  right_chassis.setStopping(brake);   // Prevents drifting, smooth control
  outtake_motor.setStopping(hold);    // Holds position, prevents backdriving
  intake_motor.setStopping(hold);     // Prevents backdriving when stopped
  
  // Initialize pneumatic states
  midscoring_piston.set(false);
  
  // odom tracking
  resetChassis();
  if(using_horizontal_tracker && using_vertical_tracker) {
    thread odom = thread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    thread odom = thread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    thread odom = thread(trackYOdomWheel);
  } else {
    thread odom = thread(trackNoOdomWheel);
  }
}