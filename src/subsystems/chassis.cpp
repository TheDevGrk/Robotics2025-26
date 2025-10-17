#include "subsystems/chassis.hpp"
#include "pros/colors.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <chrono>
#include <cmath>
#include <ctime>
#include <math.h>


// pre-processor macros
#define forever while (true)
#define sind(x) float(sin(M_PI/180.0*(x)))
#define cosd(x) float(cos(M_PI/180.0*(x)))
#define tand(x) float(tan(M_PI/180.0*(x)))

// Declare Devices
pros::MotorGroup left({12, -17, -20});
pros::MotorGroup right({-19, 11, 3});
pros::IMU imu(1);
pros::Rotation vertOdom(-14);
pros::Rotation horztOdom(-18);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

const float wheelDiameter = 2.75; //odom wheel diameter in inches
const float wheelCircumference = wheelDiameter * M_PI; //odom wheel circumference in inches
const float horztOffset = -3.0; // offset of the horizontal odom wheel from the tracking center in inches
const float vertOffset = 0.75; // offset of the vertical odom wheel from the tracking center in inches

float rXPos = 0.0;
float rYPos = 0.0;
float theta = 0.0;
float horzt = 0.0;
float vert = 0.0;
float prevTheta = 0.0;
float prevHorzt = 0.0;
float prevVert = 0.0;
float aXPos = 0.0;
float aYPos = 0.0;



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
    // define PID constants
    float kP = 1.723;
    float kD = 10.555;
    float kI = 0.03;
    
    float error = 0; // angular distance from target
    float prevError = 0;
    float integral = 0;
    float derivative = 0;
    float initialAngle = imu.get_rotation();

    float output = 0 ;
    float prevOutput = 0;

    auto startTime = std::chrono::high_resolution_clock::now();

    //lists
    std::vector<int> errorHistory; //keep track of error over time
    std::vector<float> powerHistory; //keep track of motor power over time
    int currentTime = 0; //keep track of time over time (wow!)
    setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    forever{
        float currentDistance = initialAngle - imu.get_rotation();
        error = turnDistance - currentDistance;

        // enables integral if between -10 and 10 degree error
        if (error < 15 && error > -15){
            integral += error; //calculate integral aka area underneath the error vs time graph
        }

        derivative = error - prevError; //calculate derivative aka instantaneous ROC at of error vs time graph

        // calculate motor voltage
        output = (kP * error) + (kI * integral) + (kD * derivative);

        //clamp output
        if (output > 127) output = 127;
        if (output < -127) output = -127;


        // exit pid loop if within acceptable error
        if ((error > -2.5 && error < 2.5 && derivative < 0.3 && derivative > -0.3) || std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() > 1){
            controller.clear();
            break;
        }

        prevError = error;

        left.move(-output);
        right.move(output);

        //update histories and current time
        errorHistory.push_back(error);
        powerHistory.push_back(std::abs(output));
        currentTime += 20;

        //graph the PIDs 
        graphPID(errorHistory, powerHistory, turnDistance, error, currentTime);

		controller.print(0, 0, "IMU: %f", imu.get_heading());
		controller.print(1, 0, "Dis: %f", currentDistance);
		controller.print(2, 0, "Err: %f", error);

        pros::delay(15);
    }

    left.brake();
    right.brake();
    setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
}

float angularPIDOutput(int turnDistance){
    // define PID constants
    float kP = 1.723;
    float kD = 10.555;
    float kI = 0.03;
    
    float error = 0; // angular distance from target
    float prevError = 0;
    float integral = 0;
    float derivative = 0;
    float initialAngle = imu.get_rotation();

    float output = 0 ;

    float currentDistance = initialAngle - imu.get_rotation();
    error = turnDistance - currentDistance;

    // enables integral if between -10 and 10 degree error
    if (error < 15 && error > -15){
        integral += error; //calculate integral aka area underneath the error vs time graph
    }

    derivative = error - prevError; //calculate derivative aka instantaneous ROC at of error vs time graph

    // calculate motor voltage
    output = (kP * error) + (kI * integral) + (kD * derivative);

    //clamp output
    if (output > 127) output = 127;
    if (output < -127) output = -127;


    return output;
}

void lateralPID(double driveDistance){//PID to control lateral motions, takes driveDistance is how far to drive in inches (+ forward) relative to the robot
    // pid constants
    float kP = 11.03;
    float kI = 0.0;
    float kD = 49.093;

    float error = 0; // lateral distance from target
    float prevError = 0;
    float integral = 0;
    float derivative = 0;
    float initialPose = (vertOdom.get_position() / 36000.0) * wheelCircumference;

    auto startTime = std::chrono::high_resolution_clock::now();

    float output = 0 ;
    float prevOutput = 0;

    //lists for
    std::vector<int> errorHistory; //keep track of error over time
    std::vector<float> powerHistory; //keep track of motor power over time
    int currentTime = 0; //keep track of time over time (wow!)

    forever{
        // best to calculate distance traveled using odom wheels
        float currentDistance = ((vertOdom.get_position() / 36000.0) * wheelCircumference) - initialPose;
        error = driveDistance - currentDistance;

        // enables integral if between -200 and 200 degree error
        if (error < 2 && error > -2){
            integral += error; //calculate integral aka area underneath the error vs time graph
        }

        derivative = error - prevError; //calculate derivative aka instantaneous ROC at of error vs time graph

        // calculate motor voltage
        output = (kP * error) + (kI * integral) + (kD * derivative);

        //clamp output
        if (output > 127) output = 127;
        if (output < -127) output = -127;


        // exit pid loop if within acceptable error
        if ((error > -.25 && error < .25  && derivative < .25 && derivative > -.25) || std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() > 1){
            break;
        }

        prevError = error;

        left.move(output);
        right.move(output);

        //update histories and current time
        errorHistory.push_back(error);
        powerHistory.push_back(std::abs(output));
        currentTime += 20;

        //graph the PIDs 
        graphPID(errorHistory, powerHistory, driveDistance, error, currentTime);

		controller.print(0, 0, "Err: %f", error);

        pros::delay(15);
    }

    left.brake();
    right.brake();
}

float lateralPIDOutput(double driveDistance){//PID to control lateral motions, takes driveDistance is how far to drive in inches (+ forward) relative to the robot
    // pid constants
    float kP = 11.03;
    float kI = 0.0;
    float kD = 49.093;

    float error = 0; // lateral distance from target
    float prevError = 0;
    float integral = 0;
    float derivative = 0;
    float initialPose = (vertOdom.get_position() / 36000.0) * wheelCircumference;

    auto startTime = std::chrono::high_resolution_clock::now();

    float output = 0 ;
    // best to calculate distance traveled using odom wheels
    float currentDistance = ((vertOdom.get_position() / 36000.0) * wheelCircumference) - initialPose;
    error = driveDistance - currentDistance;

    // enables integral if between -200 and 200 degree error
    if (error < 2 && error > -2){
        integral += error; //calculate integral aka area underneath the error vs time graph
    }

    derivative = error - prevError; //calculate derivative aka instantaneous ROC at of error vs time graph

    // calculate motor voltage
    output = (kP * error) + (kI * integral) + (kD * derivative);

    //clamp output
    if (output > 127) output = 127;
    if (output < -127) output = -127;

    return output;
}

void setBrakeMode(pros::motor_brake_mode_e mode){
    left.set_brake_mode_all(mode);
    right.set_brake_mode_all(mode);
}
void arcadeDrive(int lateral, int angular){
    int leftPower = lateral - angular;
    int rightPower = lateral + angular;
    int max = std::max(leftPower, rightPower);

    // normalizes power outputs if either would exceed the max voltage of the motors
    if (max > 127){
        leftPower *= (127 / max);
        rightPower *= (127 / max);
    }

    left.move(leftPower);
    right.move(rightPower);
}

void resetPose(){ //resets all values, to be called before odom starts
    vertOdom.reset_position();
	horztOdom.reset_position();

    theta = imu.get_rotation();
    vert = 0.0;
    horzt = 0.0;

    prevTheta = theta;
    prevHorzt = horzt;
    prevVert = vert;

    aXPos = 0.0;
    aYPos = 0.0;
    rXPos = 0.0;
    rYPos = 0.0;
}

void updatePose(){ //updates odometry pose (x, y, theta)
    forever{
        // update values
        theta = imu.get_rotation();
        vert = (vertOdom.get_position() / 36000.0);
        horzt = (horztOdom.get_position() / 36000.0);

        // find changes in values since last iteration
        float imuChange = theta - prevTheta;
        float horztChange = horzt - prevHorzt;
        float vertChange = vert - prevVert;

        // calculates the relative position of the bot, accounting for drift
        rYPos = (vertChange * wheelCircumference)  - (M_PI/180.0)*imuChange*vertOffset;
        rXPos = (horztChange * wheelCircumference)  - (M_PI/180.0)*imuChange*horztOffset;

        // average angle during the motion (aka average angle since the last iteration), works better than using the initial or final angle
        float angleGuess = theta + 0.5*imuChange; 

        // OLD DEBUG CODE - keeping just incase
        // printf("DEBUG: aXPos=%.6f, aYPos=%.6f (before calc)\n", aXPos, aYPos);
        // printf("DEBUG: rXPos=%.6f, rYPos=%.6f\n", rXPos, rYPos);
        // printf("DEBUG: angleGuess=%.6f\n", angleGuess);
        // printf("DEBUG: cosd(-angleGuess)=%.6f\n", cosd(-angleGuess));
        // printf("DEBUG: sind(-angleGuess)=%.6f\n", sind(-angleGuess));

        //converts the change in relative x and y coordinates to absolute coordinates using a standard rotation matrix (matrices multiplied out) and adds them to the current coordinates
        aXPos += (-rXPos*cosd(-angleGuess) - rYPos*sind(-angleGuess)); 
        aYPos += (rXPos*sind(-angleGuess) - rYPos*cosd(-angleGuess)); 


        // set prev values
        prevTheta = theta;
        prevHorzt = horzt;
        prevVert = vert;

        // OLD DEBUG CODE - keeping just incase
        // std::array<float, 14> odomValues = {prevTheta, prevHorzt, prevVert,
        //      theta, float(cosd(-angleGuess)), float(horztOdom.get_position()), imuChange, horztChange, vertChange, rYPos, rXPos, angleGuess, aXPos, aYPos};   
        // for (int i = 0; i < odomValues.size(); i++) {
        // 			printf("%.3f ", odomValues[i]);  // .3f for 3 decimal places
        // }
        // printf("\n");
        // return odomValues;
        pros::delay(10);
    }
}

float angleFrom(float angle1, float angle2){//returns angle frm -180 to 180 that is between the two given angles
    float difference = angle2 - angle1;

    return fmod(difference, 360);
}

float distance(float x1, float y1, float x2, float y2){//returns the diagonal distance from point (x1, y1) to (x2, y2) using pythagorean theorum
    return pow((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1),.5);
}

void turnToPoint(float x, float y){// turns robot to face the specified point field
    float desiredAngle = atan2(y, x) * (180/M_PI); // returns angle between -180 and 180 degrees
    float eT = angleFrom(theta, desiredAngle);
    angularPID(eT);
}

void moveToPoint(float x, float y){// moves the robot to the specified point on the field
    float desiredAngle = atan2(y - aYPos, x - aXPos) * (180/M_PI); // returns angle between -180 and 180 degrees
    float eT = 0.0; //angular error
    float eL = 0.0; //lateral error
    float maxError = 1.5; //max error allowed for the robot to stop at

    float angularOutput = 0.0;
    float lateralOutput = 0.0;

    while (eL > maxError) {
        //calculate errors
        eT = angleFrom(theta, desiredAngle); //theta error in degrees
        eL = distance(aXPos, aYPos, x, y); //lateral error in inches

        if (eL < 3){ //disable turning if within a certain lateral error to avoid robot not settling
            eT = 0;
        }

        // scales the lateral error by the cos of the angular error, effectively dampening the lateral output if the heading is too far from the target heading but still allowing lateral movement
        eL *= cosd(eT);

        if (abs(eT) > 90){
            eL = 0;
        }

        // calculate output using pids
        lateralOutput = lateralPIDOutput(eL);
        angularOutput = angularPIDOutput(eT);

        arcadeDrive(lateralOutput, angularOutput);
        pros::delay(15);
    }


}