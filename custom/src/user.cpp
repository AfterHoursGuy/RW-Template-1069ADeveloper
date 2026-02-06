#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"
#include "../custom/include/robot-config.h"

// Modify autonomous, driver, or pre-auton code below

void runAutonomous() {
  int auton_selected = 3;
  switch(auton_selected) {
    case 1:
      exampleAuton();
      break;
    case 2:
      exampleAuton2();
      break;  
    case 3:
      skills();
      break;
    case 4:
      soloawp();
      break; 
    case 5:
      rightwing();
      break;
    case 6:
      leftwing();
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
bool rearLiftState = false;

static bool intakeToggle = false;
bool btnPrev = false;
int pressStart = 0;
bool longPress = false;
bool intaken = false;
// R1 long-press control
bool r1Prev = false;
int r1PressStart = 0;
bool r1LongPress = false;
static int stored = 0;

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

    // R1 press detection (short vs long)
    if (r1 && !r1Prev) {
      r1PressStart = Brain.timer(msec);
      r1LongPress = false;
    }
    if (r1 && !r1LongPress && Brain.timer(msec) - r1PressStart > 300) {
      r1LongPress = true;
    }

    // Apply motor behavior
    if (longPress && btn) {
      // L1 long-press: reverse while holding
      front_intake.spin(reverse, 12, voltageUnits::volt);
      scoring.spin(reverse, 12, voltageUnits::volt);

    } else if (intakeToggle) {
      static bool prev_lower = false;
      static bool prev_upper = false;
      static bool prev_intake_toggle = false;
      // state: 0 = wait/run back intake until lower sees block
      //        1 = stop back intake, keep scoring forward until upper sees block
      //        2 = reverse scoring until both sensors empty -> go back to 0
      static int intake_state = 0;

      // initialize state when toggled on
      if (intakeToggle && !prev_intake_toggle) {
        intake_state = 0;
      }

      // keep front intake running (if desired)
      front_intake.spin(forward, 12, voltageUnits::volt);

      bool lower = lower_diverter_sensor.isNearObject();
      bool upper = diverter_sensor.isNearObject();
      bool lower_rising = lower && !prev_lower;
      bool upper_rising = upper && !prev_upper;

      if (intake_state == 0) {
        // run scoring forward and back intake until lower diverter sees block
        scoring.spin(forward, -12, voltageUnits::volt);
        back_intake.spin(forward, -12, voltageUnits::volt);

        if (lower_rising) {
          wait(50, msec); // small delay to allow block to settle
          back_intake.stop(coast);
          intake_state = 1;
        }

      } else if (intake_state == 1) {
        // keep scoring forward; wait for upper diverter to see a block
        scoring.spin(forward, -12, voltageUnits::volt);

        if (upper_rising) {
          // begin reversing scoring to clear sensors
          intake_state = 2;
        }

      } else { // intake_state == 2
        // reverse scoring until both sensors are empty, then restart cycle
        scoring.spin(forward, 12, voltageUnits::volt);

        if (!lower && !upper) {
          intake_state = 0;
        }
      }

      prev_lower = lower;
      prev_upper = upper;
      prev_intake_toggle = intakeToggle;

    } else if (r1LongPress && r1) {
      // R1 held long: run back intake and reverse scoring (also clear stored)
      front_intake.spin(forward, 12, voltageUnits::volt);
      scoring.spin(forward, 12, voltageUnits::volt);
      back_intake.spin(forward, 12, voltageUnits::volt);
      if (!middleGoalState) rear_lift.set(true);
      stored = 0;

    } else if (r1) {
      // R1 short press / hold (default): open hood and spin front intake
      hood.set(true);
      front_intake.spin(forward, 12, voltageUnits::volt);
      scoring.spin(forward, 12, voltageUnits::volt);
      if (!middleGoalState) rear_lift.set(true);

    } else if (r2) {
      rear_lift.set(true);
      scoring.spin(reverse, 12, voltageUnits::volt);
      back_intake.spin(reverse, 12, voltageUnits::volt);
      front_intake.stop(hold);
      stored = 0;

    } else if (!intaken && !r2 && !r1) {
      rear_lift.set(false);
      hood.set(false);
      front_intake.stop(coast);
      scoring.stop(coast);
      back_intake.stop(coast);
    }

    btnPrev = btn;
    r1Prev = r1;

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

    static bool l2Prev = false;
    if (l2 && !l2Prev) {
        scraperState = !scraperState;
        scraper.set(scraperState);
    }
    l2Prev = l2;

    

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

  diverter_sensor.setLightPower(100); // set diverter optical sensor light power to max
  lower_diverter_sensor.setLightPower(100); // set lower diverter optical sensor light power to max
}