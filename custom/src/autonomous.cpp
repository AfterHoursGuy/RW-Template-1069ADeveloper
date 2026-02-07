#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }

void exampleAuton() {
  // Use this for tuning linear and turn pid
  driveTo(60, 3000);
  turnToAngle(90, 2000);
  turnToAngle(135, 2000);
  turnToAngle(150, 2000);
  turnToAngle(160, 2000);
  turnToAngle(165, 2000);
  turnToAngle(0, 2000);
  driveTo(-60, 3000);
}

void exampleAuton2() {
  moveToPoint(24, 24, 1, 2000, false);
  moveToPoint(48, 48, 1, 2000, true);
  moveToPoint(24, 24, -1, 2000, true);
  moveToPoint(0, 0, -1, 2000, true);
  correct_angle = 0;
  driveTo(24, 2000, false, 8);
  turnToAngle(90, 800, false);
  turnToAngle(180, 800, true);
}

void skills() {
  resetChassis();
  inertial_sensor.setRotation(270, degrees);
  correct_angle = 270;
  wait(100, msec);
  resetPositionFront();
  resetPositionLeft();
  wait(100, msec);
  scoring.spin(forward, 12, voltageUnits::volt);
  front_intake.spin(forward, 12, voltageUnits::volt);
  lift.set(true);
  driveTo(33, 1200);
  turnToAngle(180, 1000);
  scraper.set(true);
  driveToWall(5, 1800, 1500, true, 12);
  resetPositionFront();
  resetPositionRight();
  boomerang(-57, -18, -1, 180, 0.4, 2500, true, 8);
  scoring.stop();
  front_intake.stop();
  scraper.set(false);
  lift.set(false);
  turnToAngle(225, 1000);
  back_intake.spin(forward, 12, voltageUnits::volt);
  driveTo(-32, 3000, true, 4);
  turnToAngle(180, 1000);
  lift.set(true);
  wing.set(true);
  driveTo(-32, 3000, true, 8);
  turnToAngle(90, 1000);
  driveTo(-12, 3000, true, 8);
  turnToAngle(0, 1000);
  resetPositionFront();
  resetPositionLeft();
  moveToPoint(-48, 29, -1, 1100, true, 8);
  back_intake.stop();
  hood.set(true);
  front_intake.spin(forward, 12, voltageUnits::volt);
  scoring.spin(forward, 12, voltageUnits::volt);
  wait(2000, msec);
  scraper.set(true);
  hood.set(false);
  resetPositionFront();
  resetPositionLeft();
  moveToPoint(-45.5, 55, 1, 2000, false, 8);
  driveToWall(5, 1500, 1000, true, 7);
  resetPositionFront();
  resetPositionLeft();
  moveToPoint(-48, 28, -1, 1100, true, 8);
  turnToAngle(0, 1000);
  hood.set(true);
  front_intake.spin(forward, 12, voltageUnits::volt);
  scoring.spin(forward, 12, voltageUnits::volt);
  wait(500, msec);
  back_intake.spin(forward, 12, voltageUnits::volt);
  wait(1800, msec);
  hood.set(true);
  scraper.set(false);
  resetPositionFront();
  resetPositionLeft(); 
  correct_angle = 5;
  driveTo(3, 2000, false, 8);
  curveCircle(180, 12, 2000, true, 8);
  hood.set(false);
  resetPositionBack();
  resetPositionRight();
  moveToPoint(-24, -33, 1, 2000, true, 8);
  turnToAngle(225, 1000);
  moveToPoint(-16, -16.5, -1, 1100, true, 8);
  turnToAngle(225, 1000);
  hood.set(true);
  lift.set(false);
  rear_lift.set(true);
  wait(200, msec);
  scoring.spin(forward, 12, voltageUnits::volt);
  front_intake.spin(forward, 12, voltageUnits::volt);
  back_intake.spin(forward, 12, voltageUnits::volt);
  wait(2000, msec);
  lift.set(true);
  rear_lift.set(false);
  hood.set(false);
  driveTo(8, 2000, true, 8);
  turnToAngle(90, 1000);
  resetPositionRight();
  resetPositionBack();
  moveToPoint(43, -46, 1, 2100, true, 8);
  turnToAngle(180, 1000);
  scraper.set(true);
  resetPositionFront();
  resetPositionLeft();
  moveToPoint(47, -55, 1, 2000, true, 8);
  driveToWall(5, 1500, 1000, true, 12);
  resetPositionFront();
  resetPositionLeft();
  scoring.stop();
  front_intake.stop();
  lift.set(false);
  wing.set(false);
  boomerang(61, -18, -1, 180, 0.4, 2500, true, 8);
  scraper.set(false);
  lift.set(false);
  wing.set(false);
  turnToAngle(135, 1000);
  driveTo(-32, 2000, true, 4);
  turnToAngle(180, 1000);
  lift.set(true);
  wing.set(true);
  driveTo(-32, 3000, true, 8);
  turnToAngle(270, 1000);
  driveTo(-9, 3000, true, 8);
  turnToAngle(0, 1000);
  resetPositionFront();
  resetPositionRight();
  moveToPoint(48, 29, -1, 1100, true, 8);
  hood.set(true);
  front_intake.spin(forward, 12, voltageUnits::volt);
  scoring.spin(forward, 12, voltageUnits::volt);
  wait(2000, msec);
  scraper.set(true);
  hood.set(false);
  resetPositionFront();
  resetPositionRight();
  moveToPoint(45.5, 55, 1, 2000, false, 8);
  driveToWall(5, 1500, 1000, true, 7);
  resetPositionFront();
  resetPositionRight();
  moveToPoint(48, 28, -1, 1100, true, 8);
  turnToAngle(0, 1000);
  hood.set(true);
  front_intake.spin(forward, 12, voltageUnits::volt);
  scoring.spin(forward, 12, voltageUnits::volt);
  wait(500, msec);
  back_intake.spin(forward, 12, voltageUnits::volt);
  wait(1800, msec);
  hood.set(true);
  scraper.set(false);
  resetPositionFront();
  resetPositionRight(); 
  correct_angle = 355;
  driveTo(5, 2000, false, 8);
  curveCircle(180, -12, 2000, true, 8);
  resetPositionBack();
  resetPositionLeft();
  moveToPoint(24, -58, 1, 3000, true, 12);
  wait(100, msec);
  scraper.set(false);
  lift.set(false);
  swing(270, 1, 2000, true, 12);
  odom_lift.set(false);
  driveTo(39, 10000, true, 8);

  

  



  /*resetChassis();
  inertial_sensor.setRotation(270, degrees);
  correct_angle = 270;
  lift.set(true);
  wing.set(true);
  odom_lift.set(true);
  front_intake.spin(forward, 12, voltageUnits::volt);
  scoring.spin(forward, 12, voltageUnits::volt);
  driveChassis(7.5, 7.5);
  wait(600, msec);
  driveChassis(0, 0);
  wait(200, msec);
  driveChassis(7, 7.8);
  wait(900, msec);
  scraper.set(true);
  wait(400, msec);
  driveChassis(0, 0);
  scraper.set(true);
  wait(200, msec);
  odom_lift.set(false);
  wait(100, msec);
  correct_angle = 270;
  driveTo(-8, 1000);
  turnToAngle(180, 1000);
  driveTo(-4, 800, true, 8);
  scraper.set(false);
  wait(100, msec);
  resetPositionFront();
  resetPositionRight();
  wait(50, msec);
  back_intake.spin(forward, 12, voltageUnits::volt);
  boomerang(-24, -24, -1, 0, 0.2, 2000, true, 6);
  wait(50, msec);
  resetPositionFront();
  resetPositionRight();
  wait(50, msec);
  turnToAngle(225, 1000);
  wait(50, msec);
  boomerang(-10, -11, -1, 225, 0.2, 4000, true, 6);
  scraper.set(false);
  lift.set(false);
  wait(130, msec);
  back_intake.spin(forward, 12, voltageUnits::volt);
  hood.set(true);
  rear_lift.set(true);
  wait(2100, msec);
  boomerang(-40, -42, 1, 180, 0.2, 4000, true, 8);
  turnToAngle(180, 1000);
  wait(200, msec);
  resetPositionFront();
  resetPositionRight();
  rear_lift.set(false);
  lift.set(true);
  wait(200, msec);
  scraper.set(true);
  moveToPoint(-47.5, -55, 1, 2000, true, 8);
  turnToAngle(180, 1000);
  hood.set(false);
  wait(300, msec);
  driveToWall(5, 1500, 1000, true, 7);
  resetPositionFront();
  resetPositionRight();
  boomerang(-29, -28, -1, 180, 0.2, 900, false, 8);
  scraper.set(false);
  boomerang(-24, 28, -1, 180, 0.2, 1200, true, 8);
  wait(200, msec);
  resetPositionBack();
  resetPositionRight();
  wait(50, msec);
  boomerang(-48, 48, -1, 180, 0.4, 2500, true, 8);
  wait(300, msec);
  turnToAngle(0, 1000);
  resetPositionFront();
  resetPositionLeft();
  moveToPoint(-48, 28, -1, 1100, true, 8);
  turnToAngle(0, 1000);
  hood.set(true);
  front_intake.spin(forward, 12, voltageUnits::volt);
  scoring.spin(forward, 12, voltageUnits::volt);
  resetPositionFront();
  resetPositionLeft();
  wait(2000, msec);
  scraper.set(true);
  hood.set(false);
  moveToPoint(-45.5, 55, 1, 2000, false, 8);
  driveToWall(5, 1500, 1000, true, 7);
  resetPositionFront();
  resetPositionLeft();
  moveToPoint(-48, 28, -1, 1100, true, 8);
  turnToAngle(0, 1000);
  hood.set(true);
  front_intake.spin(forward, 12, voltageUnits::volt);
  scoring.spin(forward, 12, voltageUnits::volt);
  wait(500, msec);
  back_intake.spin(forward, 12, voltageUnits::volt);
  wait(1800, msec);
  resetPositionFront();
  resetPositionLeft();*/

}

void soloawp() {
  resetChassis();
  inertial_sensor.setRotation(270, degrees);
  correct_angle = 270;
  resetPositionBack();
  resetPositionLeft();
  wait(50, msec);
  scoring.spin(forward, 12, voltageUnits::volt);
  front_intake.spin(forward, 12, voltageUnits::volt);
  lift.set(true);
  driveTo(6, 1000, true, 12);
  wait(200, msec);
  resetPositionBack();
  resetPositionLeft();
  thread([]() {
    wait(600, msec);
    scraper.set(true);
  }).detach();
  moveToPoint(41.5, -48, -1, 2000, true, 10);
  turnToAngle(180, 1000);
  driveToWall(5.5, 1500, 0, true, 12);
  resetPositionFront();
  resetPositionLeft();
  moveToPoint(47, -29, -1, 1000, true, 10);
  resetPositionFront();
  resetPositionLeft();
  scraper.set(false);
  hood.set(true);
  wait(800, msec);
  scoring.stop();
  turnToAngle(270, 1000);
  hood.set(false);
  scoring.spin(forward, 12, voltageUnits::volt);
  resetPositionLeft();
  resetPositionBack();
  moveToPoint(24, -25, 1, 3000, false, 10);
  thread([]() {
    wait(1000, msec);
    scraper.set(true);
    wait(150, msec);
    scraper.set(false);
  }).detach();
  moveToPoint(-26, -25, 1, 3000, true, 10);
  turnToAngle(270, 600);
  resetPositionLeft();
  resetPositionFront();
  turnToAngle(225, 1000);
  moveToPoint(-10, -12, -1, 1100, true, 10);
  turnToAngle(225, 1000);
  hood.set(true);
  lift.set(false);
  rear_lift.set(true);
  wait(450, msec);
  hood.set(false);
  lift.set(true);
  rear_lift.set(false);
  boomerang(-39, -48, 1, 180, 0.2, 4000, true, 10);
  turnToAngle(180, 1000);
  resetPositionFront();
  resetPositionRight();
  wait(500, msec);
  moveToPoint(-48, -30, -1, 1100, true, 10);
  hood.set(true);
 


}

void rightwing() {
  lift.set(true);
  front_intake.spin(forward, 12, voltageUnits::volt);
  scoring.spin(forward, 12, voltageUnits::volt);
  back_intake.spin(forward, 12, voltageUnits::volt);
  thread([]() {
    wait(700, msec);
    scraper.set(true);
  }).detach();
  moveToPoint(5.5, 29, 1, 3000, true, 8);
  swing(120, -1, 2000, true, 10);
  scraper.set(false);
  moveToPoint(40, 5, 1, 2000, true, 8);
  turnToAngle(180, 1000);
  wait(200, msec);
  resetPositionFront();
  resetPositionLeft();
  wait(200, msec);
  boomerang(47, -28, -1, 180, 0.2, 2000, true, 8);
  hood.set(true);
  turnToAngle(180, 1000);
  wait(800, msec);
  curveCircle(90, -12, 2000, true, 8);
  driveTo(1, 2000, true, 8);
  turnToAngle(180, 1000);
  hood.set(false);
  driveTo(-36, 2000, true, 12);
}

void leftwing() {
  lift.set(true);
  front_intake.spin(forward, 12, voltageUnits::volt);
  scoring.spin(forward, 12, voltageUnits::volt);
  back_intake.spin(forward, 12, voltageUnits::volt);
  thread([]() {
    wait(800, msec);
    scraper.set(true);
  }).detach();
  moveToPoint(-5.5, 28, 1, 3000, true, 8);
  swing(240, -1, 2000, true, 10);
  scraper.set(false);
  moveToPoint(-23, 5, 1, 2000, true, 8);
  turnToAngle(180, 1000);
  wait(200, msec);
  resetPositionFront();
  resetPositionRight();
  wait(200, msec);
  boomerang(-48, -31, -1, 180, 0.2, 2000, true, 8);
  hood.set(true);
  turnToAngle(180, 1000);
  wait(800, msec);
  curveCircle(90, -12, 2000, true, 8);
  turnToAngle(180, 1000);
  hood.set(false);
  driveTo(-36, 2000, true, 12);


}