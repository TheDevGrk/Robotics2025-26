#include "subsystems/chassis.hpp"
#include "pros/colors.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"


// pre-processor macros
#define forever while (true)

// Declare Devices
pros::MotorGroup left({12, -17, -20});
pros::MotorGroup right({-19, 11, 3});
pros::IMU imu(1);
pros::Rotation vertOdom(-14);
pros::Rotation horztOdom(-18);

void arcadeDrive(int lateral, int angular){
    left.move(lateral - angular);
    right.move(lateral + angular);
}

//graphing data, used for PID tuning
void graphPID(std::vector<int> errorHistory, std::vector<float> powerHistory, int goal, float error, int time) {
  //goal is the PID goal (driveDistance)
  //error history is a list of all of the errors (range is 0 to driveDistance)
  //powerHistory is a list of the power applied (range is -1 to 1)
  //error is the current error
  //time is the current time, in milliseconds
  
  //Setup: clear screen and draw the target line
  pros::screen::erase();
  pros::screen::set_pen(pros::Color::white);
  pros::screen::draw_line(0, 60, 480, 60);
  pros::screen::set_pen(pros::Color::green);

  //display final error and time
  pros::screen::print(pros::E_TEXT_MEDIUM,  1, " Final Error: %f    TIme: %i", error, time);
  
  //define the borders of the graph
  int minY = 60; //error = 0 (robot is at target)
  int maxY = 230; //error = driveDistance (Robot is at start)
  int minX = 10; //time = beginning
  int maxX = 470; //time = end
  
  //loop through each data point and graph it
  for (int i = 0; i < errorHistory.size() - 1; i++) { 
    int x = minX + (maxX - minX) * i / errorHistory.size(); //find the x-value of this data point
    
    //graph velocity
    pros::screen::set_pen(pros::Color::green);
    pros::screen::draw_line(x, minY + (float)errorHistory.at(i) / goal * (maxY - minY), x + (float)(maxX - minX) / errorHistory.size(), minY + (float)errorHistory.at(i + 1) / goal * (maxY - minY));
    
    //graph power, changing color based on direction
    if (powerHistory.at(i) > 0) {
        pros::screen::set_pen(pros::Color::orange);
    } else {
        pros::screen::set_pen(pros::Color::yellow);
    }

    pros::screen::draw_line(x, maxY - std::abs(powerHistory.at(i)) * (maxY - minY), x + (float)(maxX - minX) / errorHistory.size(), maxY - std::abs(powerHistory.at(i + 1)) * (maxY - minY));
    
    }
}

void angularPID(int turnDistance){//PID to control angular motions, takes turnDistance is how far to turn (+ clockwise)
    //define pid constants
    float kP = 0.5;
    float kI = 0;
    float kD = 0;

    float error = 0; // angular distance from target
    float prevError = 0;
    float integral = 0;
    float derivative = 0;
    float initialAngle = imu.get_heading();

    float output = 0 ;
    float prevOutput = 0;

    //lists
    std::vector<int> errorHistory; //keep track of error over time
    std::vector<float> powerHistory; //keep track of motor power over time
    int currentTime = 0; //keep track of time over time (wow!)

    forever{
        float currentDistance = initialAngle - imu.get_heading();
        error = turnDistance - currentDistance;

        // enables integral if between -10 and 10 degree error
        if (error < 10 && error > -10){
            integral += error; //calculate integral aka area underneath the error vs time graph
        }

        derivative = error - prevError; //calculate derivative aka instantaneous ROC at of error vs time graph

        // calculate motor voltage
        output = (kP * error) + (kI * integral) + (kD * derivative);

        //clamp output
        if (output > 127) output = 127;
        if (output < -127) output = -127;


        // exit pid loop if within acceptable error
        if (error > -1 && error < 1 && derivative < 0.3 && derivative > -0.3) break;

        prevError = error;

        //update histories and current time
        errorHistory.push_back(error);
        powerHistory.push_back(std::abs(output));
        currentTime += 20;

        //graph the PIDs 
        graphPID(errorHistory, powerHistory, turnDistance, error, currentTime);

        pros::delay(15);
    }

    left.brake();
    right.brake();
}

void setBrakeMode(pros::motor_brake_mode_e mode){
    left.set_brake_mode(mode);
    right.set_brake_mode(mode);

    pros::lcd::print(1, "Successfully set brake mode");
}


