#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/imu.hpp"

extern pros::MotorGroup left;
extern pros::MotorGroup right;
extern pros::IMU imu;
extern pros::Rotation vertOdom;
extern pros::Rotation horztOdom;
extern pros::Controller controller;

extern float rXPos;
extern float rYPos;
extern float theta;
extern float prevTheta;
extern float aXPos;
extern float aYPos;

void arcadeDrive(int lateral, int angular);
void angularPID(int turnDistance);
void lateralPID(double driveDistance);
void setBrakeMode(pros::motor_brake_mode_e mode);
void updatePose();
void resetPose();
void turnToPoint(float x, float y);
void moveToPoint(float x, float y);

#endif // CHASSIS_HPP
