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

void arcadeDrive(int lateral, int angular);
void angularPID(int turnDistance);
void lateralPID(double driveDistance, float kP, float kI, float kD);
void setBrakeMode(pros::motor_brake_mode_e mode);

#endif // CHASSIS_HPP
