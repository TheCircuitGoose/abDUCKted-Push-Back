// Include Libraries
#include "main.h"

#include <iostream>
#include <thread>

#include "liblvgl/lvgl.h"
#include "lemlib/api.hpp"

// Device Declarations
pros::Controller primary(pros::E_CONTROLLER_MASTER);				// Creates primary controller
pros::MotorGroup left_mg({-1, -2, 3}, pros::MotorGearset::blue);	// Creates left drive motor group with ports 1, 2, and 3
pros::MotorGroup right_mg({4, 7, -8}, pros::MotorGearset::blue);	// Creates right drive motor group with ports 4, 5, and 6
pros::MotorGroup intake_mg({9, -10});	                            // Creates intake motor group with ports 7 and 8
pros::Imu inertial(11);												// Creates inertial sensor on port 10
pros::Rotation hTrack(12);											// Creates horizontal tracking wheel on port 11
pros::Rotation vTrack(13);                                          // Creates vertical tracking wheel on port 12

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
                                      -.5 // Distance from robot center in inches (Negative for Left Side)
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
lemlib::ControllerSettings lateral_controller(300, // proportional gain (kP)
                                              20, // integral gain (kI)
                                              0, // derivative gain (kD)
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

// When Start
void initialize() {
	inertial.reset(); // Reset the inertial sensor
	hTrack.reset(); // Reset the horizontal tracking wheel
    vTrack.reset(); // Reset the vertical tracking wheel
	chassis.calibrate(); // Calibrate the chassis sensors
}

// When Disabled
void disabled() {}

// When Connected to Field Control
void competition_initialize() {}

// When Autonomous
void autonomous() {
	chassis.setPose(0, 0, 0);
	pros::delay(5000);
	chassis.moveToPose(0, 72, 0, 5000);
}

// When Driver Control
void opcontrol() {
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