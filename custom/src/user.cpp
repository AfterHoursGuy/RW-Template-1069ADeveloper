#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include "../custom/include/robot-config.h"

// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  int auton_selected = 1;
  switch(auton_selected) {
    case 1:
      exampleAuton();
      break;
    case 2:
      exampleAuton2();
      break;  
    case 3:
      break;
    case 4:
      break; 
    case 5:
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

// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;

// pneumatic control
bool scraperState = false;
bool middleGoalState = false;
bool parkPistonState = false;
bool wingState = false;

static bool intakeToggle = false;
bool btnPrev = false;
int pressStart = 0;
bool longPress = false;
bool intaken = false;

void runDriver() {
  stopChassis(coast);
  heading_correction = false;
  odom_lift.set(true); // lift odom wheels for driving
  while (true) {
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // true/false for controller button presses
    l1 = controller_1.ButtonL1.pressing();
    l2 = controller_1.ButtonL2.pressing();
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_x = controller_1.ButtonX.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();

    // default tank drive or replace it with your preferred driver code here: 
    driveChassis(ch3 * 0.12, ch2 * 0.12);

    bool btn = controller_1.ButtonL1.pressing();

    // Button pressed this moment
    if (btn && !btnPrev) {
        pressStart = Brain.timer(msec);
        longPress = false;
    }

    // If held long enough → reverse
    if (btn && !longPress && Brain.timer(msec) - pressStart > 150) {
        longPress = true;
    }

    // Handle release
    if (!btn && btnPrev) {
        if (!longPress) {
            // short press → toggle
            intakeToggle = !intakeToggle;
        }
        // if longPress: do nothing → return to toggle state
    }

    // Apply motor behavior
    if (longPress && btn) {
        // reverse while holding long press
        front_intake.spin(reverse, 12, voltageUnits::volt);
        scoring.spin(reverse, 12, voltageUnits::volt);
    }
    else if (intakeToggle) {
        // normal toggle state
        front_intake.spin(forward, 12, voltageUnits::volt);
        scoring.spin(forward, 12, voltageUnits::volt);

    } else if (r1) {
        hood.set(true);
        front_intake.spin(forward, 12, voltageUnits::volt);
        scoring.spin(forward, 12, voltageUnits::volt);

    } else if (!intaken && !r2) {
        hood.set(false);
        front_intake.stop(coast);
        scoring.stop(coast);
    }

    btnPrev = btn;

    // Right arrow toggle for Middle goal
    static bool rightPrev = false;
    if (button_right_arrow && !rightPrev) {
    middleGoalState = !middleGoalState;
    lift.set(middleGoalState);
    }
    rightPrev = button_right_arrow;

    static bool yPrev = false;
    if (button_y && !yPrev) {
    wingState = !wingState;
    wing.set(wingState);
    }
    yPrev = button_y;

    if (r2) {
        back_intake.spin(reverse, 12, voltageUnits::volt);
    } else {
        back_intake.stop(coast);
    }

    wait(10, msec); 
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