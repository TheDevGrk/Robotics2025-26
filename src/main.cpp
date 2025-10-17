#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "subsystems/chassis.hpp"

// Simple config variables
float configkP = 1.5f;
float configkI = 0.0f;
float configkD = 11.0f;
float increment = 1.0f;
std::string currentlyEditingConstant = "kP";

pros::Motor intake(-5);

// Simple function to save config
void saveConfig() {
    FILE* file = fopen("/usd/pid_config.txt", "w");
    if(file) {
        fprintf(file, "kP=%.3f\nkI=%.3f\nkD=%.3f\nincrement=%.4f\ncurrentlyEditingConstant=%s\n", 
                configkP, configkI, configkD, increment, currentlyEditingConstant.c_str());
        fclose(file);
        printf("Config saved!\n");
    }
}

// Simple function to load pid config
void loadConfig() {
    FILE* file = fopen("/usd/pid_config.txt", "r");
    if(file) {
        fscanf(file, "kP=%f\nkI=%f\nkD=%f\nincrement=%f\ncurrentlyEditingConstant=%s\n", 
               &configkP, &configkI, &configkD, &increment, currentlyEditingConstant.data());
        fclose(file);
        printf("Config loaded: kP=%.3f, kI=%.3f, kD=%.3f, increment=%.4f\n, currentlyEditingConstant=%s\n", configkP, configkI, configkD, increment, currentlyEditingConstant.c_str());
    }
}


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	imu.reset();
	while(imu.is_calibrating()) {
        pros::delay(50);
    }

	resetPose();
	setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	
	// Load simple config
	loadConfig();
	saveConfig();

	// Create thread for odometry
	pros::Task odometry(updatePose);
	

	pros::lcd::register_btn1_cb(on_center_button);
	controller.rumble("..");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		updatePose();
		// ! IDEA: Have a button that will record the specific odom point of the bot at any one time, can use it for path planning, helps if odom is consistently off by same amount (path planner would pick points that are always correct whereas if bot is always an inch off, those points won't be correct but if we choose our own points they will be)

		// if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
		// 	loadConfig();
		// 	angularPID(180);
		// 	pros::delay(150);
        //     lateralPID(15);
		// 	controller.rumble("-");
        // }
		// if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        //     // Load config from file
		// 	loadConfig();
        // }
		// if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
		// 	// Save config to file
		// 	saveConfig();
		// }
		// if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
		// 	//increment the currently selected constant
		// 	if (currentlyEditingConstant == "kP") {
		// 		configkP += increment;
		// 	} else if (currentlyEditingConstant == "kI") {
		// 		configkI += increment;
		// 	} else if (currentlyEditingConstant == "kD") {
		// 		configkD += increment;
		// 	}
		// 	saveConfig();
        // }
		// if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		// 	//decrement the currently selected constant
		// 	if (currentlyEditingConstant == "kP") {
		// 		configkP -= increment;
		// 	} else if (currentlyEditingConstant == "kI") {
		// 		configkI -= increment;
		// 	} else if (currentlyEditingConstant == "kD") {
		// 		configkD -= increment;
		// 	}
		// 	saveConfig();
		// }
		// if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
		// 	//cycle to the previous constant
		// 	if (currentlyEditingConstant == "kP") {
		// 		currentlyEditingConstant = "kD";
		// 	} else if (currentlyEditingConstant == "kI") {
		// 		currentlyEditingConstant = "kP";
		// 	} else if (currentlyEditingConstant == "kD") {
		// 		currentlyEditingConstant = "kI";
		// 	} else {
		// 		currentlyEditingConstant = "kP";
		// 	}
		// 	saveConfig();
		// }
		// if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
		// 	//cycle to the next constant
		// 	if (currentlyEditingConstant == "kP") {
		// 		currentlyEditingConstant = "kI";
		// 	} else if (currentlyEditingConstant == "kI") {
		// 		currentlyEditingConstant = "kD";
		// 	} else if (currentlyEditingConstant == "kD") {
		// 		currentlyEditingConstant = "kP";
		// 	} else {
		// 		currentlyEditingConstant = "kP";
		// 	}
		// 	saveConfig();
		// }
		// if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
		// 	//cycle increments/decrements, increments multiplied by 100 to avoid float precision issues
		// 	increment *= 100;
		// 	if (increment == 1000.0f) {
		// 		increment = 100.0f;
		// 	} else if (increment == 100.0f) {
		// 		increment = 10.0f;
		// 	} else if (increment == 10.0f) {
		// 		increment = 1.0f;
		// 	} else if (increment == 1.0f) {
		// 		increment = 0.1f;  // This represents 0.001 when divided by 100
		// 	} else if (increment == 0.1f) {
		// 		increment = 1000.0f;
		// 	} else {
		// 		increment = 1.0f;
		// 	}
		// 	increment /= 100; 
		// 	saveConfig();
		// }

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
			intake.move(127);
		}
		if (controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R2)){
			intake.brake();
		}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
			intake.move(-127);
		}
		if (controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R1)){
			intake.brake();
		}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			moveToPoint(54, 72);
		}
		

		controller.print(0, 0, "(%f, %f)", aXPos, aYPos);
		pros::screen::print(pros::E_TEXT_MEDIUM, 0, "(X: %f, Y: %f, θ: %f)", aXPos, aYPos, theta);
		
		// controller.print(1, 0, "%s: %f", currentlyEditingConstant.c_str(), 
		// 				 (currentlyEditingConstant == "kP" ? configkP : 
		// 				  currentlyEditingConstant == "kI" ? configkI : 
		// 				  currentlyEditingConstant == "kD" ? configkD : 0.0f));

		// Arcade control scheme
		int lateral = controller.get_analog(ANALOG_LEFT_Y);
		int angular = -0.8 * controller.get_analog(ANALOG_RIGHT_X);
		arcadeDrive(lateral, angular);
		pros::delay(20);
	}
}