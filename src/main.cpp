// Include Libraries
#include "main.h"

#include <iostream>
#include <thread>
#include <string>

#include "liblvgl/lvgl.h"
#include "lemlib/api.hpp"

// Device Declarations
// Ports 5, 6, and 9 are Dead =(
pros::Controller primary(pros::E_CONTROLLER_MASTER);				// Creates primary controller

pros::Motor left1(-1, pros::MotorGearset::blue);
pros::Motor left2(-2, pros::MotorGearset::blue);
pros::Motor left3(3, pros::MotorGearset::blue);
pros::Motor right1(4, pros::MotorGearset::blue);
pros::Motor right2(7, pros::MotorGearset::blue);
pros::Motor right3(-8, pros::MotorGearset::blue);

pros::MotorGroup left_mg({-1, -2, 3}, pros::MotorGearset::blue);	// Creates left drive motor group with ports 1, 2, and 3
pros::MotorGroup right_mg({4, 7, -8}, pros::MotorGearset::blue);	// Creates right drive motor group with ports 4, 5, and 6

pros::MotorGroup intake_mg({-10, 14});	                            // Creates intake motor group with ports 7 and 8
pros::Imu inertial(11);												// Creates inertial sensor on port 10
pros::Rotation hTrack(12);											// Creates horizontal tracking wheel on port 11
pros::Rotation vTrack(-13);                                          // Creates vertical tracking wheel on port 12

// LemLib Declarations
// Drivetrain Configuration
lemlib::Drivetrain drivetrain(&left_mg, // Left Motor Group
							  &right_mg, // Right Motor Group
							  12.625, // Track Width in inches (distance from left to right wheels)
							  lemlib::Omniwheel::NEW_325, // Anti-Static 3.25" Omni Wheels
							  450, // Drivetrain Speed in RPM
							  2 // Horizontal Drift (WILL BE ADJUSTED LATER)
);

// Horizontal Tracking Wheel Configuration
lemlib::TrackingWheel horizontalTrack(&hTrack, // Horizontal Tracking Wheel Rotation Sensor
									  lemlib::Omniwheel::NEW_325, // AS 3.25" Omni Wheel
									  1 // Distance from robot center in inches (Positive for forward)
);

// Vertical Tracking Wheel Configuration
lemlib::TrackingWheel verticalTrack(&vTrack, // Vertical Tracking Wheel Rotation Sensor
                                      lemlib::Omniwheel::NEW_325, // AS 3.25" Omni Wheel
                                      -.25 // Distance from robot center in inches (Negative for Left Side)
);

// Odometry Sensors Configuration
lemlib::OdomSensors sensors(&verticalTrack, // vertical tracking wheel 1, set to vertical rotation sensor.
                            nullptr, // vertical tracking wheel 2, set to null
                            &horizontalTrack, // horizontal tracking wheel 1, set to horizontal rotation sensor. 
                            nullptr, // horizontal tracking wheel 2, set to null
                            &inertial // inertial sensor, set to inertial sensor device
);

// Work in Progess
// Lateral PID Controller Configuration
lemlib::ControllerSettings lateral_controller(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              12, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

//Angular PID Controller Configuration
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

bool isLogging = false;
int targetDistance = 0;

void logTask() {
    FILE* logFile = fopen("/usd/log.csv", "a"); // Open log file on sd card for appending
    fwrite("Time,Left,Right,Avg,Error\n", 1, 27, logFile);
    int time = 0;

    while (true) {
        if (isLogging) {
            int leftVelocity = left1.get_actual_velocity(); // Get the velocity of left motor 1
            int rightVelocity = right1.get_actual_velocity(); // Get the velocity of right motor 1
            int avgVelocity = (leftVelocity + rightVelocity) / 2; // Calculate average velocity

            float distance = chassis.getPose().y; // Get the current distance from the start position
            float error = targetDistance - distance; // Calculate error from target position
            
            std::string logMessage = std::to_string(time) +
                "," + std::to_string(leftVelocity) + 
                "," + std::to_string(rightVelocity) + 
                "," + std::to_string(avgVelocity) +
                "," + std::to_string(error) + "\n";

            fwrite(logMessage.c_str(), 1, logMessage.size(), logFile);

            time += 10;
        }
        pros::delay(10);
    }
}

// When Start
void initialize() {
	inertial.reset(); // Reset the inertial sensor
	hTrack.reset(); // Reset the horizontal tracking wheel
    vTrack.reset(); // Reset the vertical tracking wheel
	chassis.calibrate(); // Calibrate the chassis sensors

    pros::Task logTaskObj(logTask);
}

// When Disabled
void disabled() {}

// When Connected to Field Control
void competition_initialize() {}

// When Autonomous
void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::delay(4750);
	chassis.setPose(0, 0, 0);
    isLogging = true; // Start logging data
	pros::delay(250);
    targetDistance = 72;
	chassis.moveToPose(0, 72, 0, 10000);
}

// When Driver Control
void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST); // Set the brake mode to brake
	while (true) {
		int left = primary.get_analog(ANALOG_LEFT_Y); // Gets Left Stick Up/Down Value
		int right = primary.get_analog(ANALOG_RIGHT_Y); // Gets Right Stick Up/Down Value
		chassis.tank(left, right);

        if (primary.get_digital(DIGITAL_L1)) {
            intake_mg.move_velocity(200);
        } else if (primary.get_digital(DIGITAL_L2)) {
            intake_mg.move_velocity(-200);
        } else {
            intake_mg.move_velocity(0);
        }
        
		pros::delay(10);
	}
}