// Include Libraries
#include "main.h"

#include <thread>
#include <string>

#include "liblvgl/lvgl.h"
#include "lemlib/api.hpp"

#include "project/auton.hpp"
#include "project/ui.hpp"

#define TORQUE_THRESHOLD 0.15

// Device Declarations
// Ports 5, 6, 9, and 13 are Dead =(
pros::Controller driver(pros::E_CONTROLLER_MASTER);				// Creates primary controller
pros::Controller partner(pros::E_CONTROLLER_PARTNER);           // Creates secondary controller

pros::Motor left1(-1, pros::MotorGearset::blue);
pros::Motor left2(-2, pros::MotorGearset::blue);
pros::Motor left3(3, pros::MotorGearset::blue);
pros::Motor right1(4, pros::MotorGearset::blue);
pros::Motor right2(7, pros::MotorGearset::blue);
pros::Motor right3(-8, pros::MotorGearset::blue);

pros::MotorGroup left_mg({-1, -2, 3}, pros::MotorGearset::blue);	// Creates left drive motor group with ports 1, 2, and 3
pros::MotorGroup right_mg({4, 7, -8}, pros::MotorGearset::blue);	// Creates right drive motor group with ports 4, 7, and 8

pros::MotorGroup intake_mg({10, -14});	                            // Creates intake motor group with ports 10 and 14
pros::Motor lower_conveyor(-15);                                     // Creates lower conveyor motor on port 15
pros::Motor upper_conveyor(-16);                                     // Creates upper conveyor motor on port 16

pros::ADIDigitalOut leftLift('A');
pros::ADIDigitalOut rightLift('B');

pros::Imu inertial(11);												// Creates inertial sensor on port 11
pros::Rotation hTrack(12);											// Creates horizontal tracking wheel on port 11
pros::Rotation vTrack(-17);                                          // Creates vertical tracking wheel on port 17

pros::Optical intake_color(18);                                   // Creates optical sensor on port 18

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
lemlib::ControllerSettings lateral_controller(5.5, // proportional gain (kP)
                                              0, // integral gain (kI)
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

// UI Declarations
int autonIndex = 0;													// Declares an int for storing the selected auton routine.
int colorIndex = 0;													// Declares an int for storing the selected color.
UI ui;

// When Start
void initialize() {
    lv_init();

    inertial.reset(); // Reset the inertial sensor
    hTrack.reset(); // Reset the horizontal tracking wheel
    vTrack.reset(); // Reset the vertical tracking wheel
    chassis.calibrate(); // Calibrate the chassis sensors

    pros::Task logTaskObj(logTask);

    inertial.reset();
    hTrack.reset();
    vTrack.reset();
    chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    ui.initUI_NEW(lv_scr_act());
}

// When Disabled
void disabled() {}

// When Connected to Field Control
void competition_initialize() {}

// When Autonomous
void autonomous() {
    autonIndex = ui.getAutonIndex();
    colorIndex = ui.getColorIndex();
    switch (autonIndex) {
        case 1: // Left
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            chassis.setPose(-50, 18, 0); // set starting position, touching parking zone
            leftLift.set_value(false);
            rightLift.set_value(false);
            pros::delay(10);
            chassis.moveToPoint(-50, 48, 3000); // drive to match loader
            chassis.turnToHeading(270, 1000);
            chassis.moveToPoint(-59, 48, 3000, {.maxSpeed = 75});
            intake_mg.move_velocity(-200); //unload
            lower_conveyor.move_velocity(200);
            if (colorIndex != 0) {
                while (get_color(intake_color) != colorIndex) { // wait until other color shows if using color sensing
                    pros::delay(10);
                }
                intake_mg.move_velocity(0);
                lower_conveyor.move_velocity(0);
            } else {
                pros::delay(2000); // wait 2 second if not use color sensing
                intake_mg.move_velocity(0);
                lower_conveyor.move_velocity(0);
            }
            chassis.moveToPose(-44, 48, 270, 2000, {.forwards = false}); // go to long goal
            pros::delay(100);
            chassis.moveToPose(-26, 48, 90, 3000);
            pros::delay(2500);
            lower_conveyor.move_velocity(200); // score
            upper_conveyor.move_velocity(200);
            intake_mg.move_velocity(-200);
            pros::delay(5000);
            lower_conveyor.move_velocity(0);
            upper_conveyor.move_velocity(0);
            intake_mg.move_velocity(0);
        case 2: // Right
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            chassis.setPose(-50, -18, 180);
            pros::delay(10);
            chassis.moveToPose(-50, -48, 270, 3000);
            chassis.moveToPose(-54, -48, 270, 3000, {.maxSpeed = 75});
            intake_mg.move_velocity(200);
            lower_conveyor.move_velocity(200);
            if (colorIndex != 0) {
                while (get_color(intake_color) != colorIndex) {
                    pros::delay(10);
                }
                intake_mg.move_velocity(0);
                lower_conveyor.move_velocity(0);
            } else {
                pros::delay(2000);
                intake_mg.move_velocity(0);
                lower_conveyor.move_velocity(0);
            }
            chassis.moveToPose(-50, -48, 270, 2000, {.forwards = false});
            leftLift.set_value(false);
            rightLift.set_value(false);
            chassis.moveToPose(-28, -48, 90, 3000);
            lower_conveyor.move_velocity(200);
            upper_conveyor.move_velocity(200);
            pros::delay(5000);
            lower_conveyor.move_velocity(0);
            upper_conveyor.move_velocity(0);
        case 3: // PID Tuning
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            pros::delay(4750);
            chassis.setPose(0, 0, 0);
            isLogging = true; // Start logging data
            pros::delay(250);
            targetDistance = 72;
            chassis.moveToPose(0, 72, 0, 10000);
    }
}

// When Driver Control
void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST); // Set the brake mode to brake
    int partnerIndex = ui.getPartnerIndex();
    if (partnerIndex == 0) {
        while (true) {
            int left = driver.get_analog(ANALOG_LEFT_Y); // Gets Left Stick Up/Down Value
            int right = driver.get_analog(ANALOG_RIGHT_Y); // Gets Right Stick Up/Down Value
            chassis.tank(left, right);

            if (driver.get_digital(DIGITAL_L1) || partner.get_digital(DIGITAL_L1)) {
                intake_mg.move_velocity(200);
            } else if (driver.get_digital(DIGITAL_L2) || partner.get_digital(DIGITAL_L2)) {
                intake_mg.move_velocity(-200);
            } else {
                intake_mg.move_velocity(0);
            }

            if (driver.get_digital(DIGITAL_R1) || partner.get_digital(DIGITAL_R1)) {
                lower_conveyor.move_velocity(200);
                upper_conveyor.move_velocity(200);
            } else if (driver.get_digital(DIGITAL_R2) || partner.get_digital(DIGITAL_R2)) {
                lower_conveyor.move_velocity(-200);
                upper_conveyor.move_velocity(-200);
            } else {
                lower_conveyor.move_velocity(0);
                upper_conveyor.move_velocity(0);
            }

            if (partner.get_digital(DIGITAL_UP)) {
                lower_conveyor.move_velocity(200);
            } else if (partner.get_digital(DIGITAL_DOWN)) {
                lower_conveyor.move_velocity(-200);
            } else {
                lower_conveyor.move_velocity(0);
            }

            if (partner.get_digital(DIGITAL_X)) {
                upper_conveyor.move_velocity(200);
            } else if (partner.get_digital(DIGITAL_B)) {
                upper_conveyor.move_velocity(-200);
            } else {
                upper_conveyor.move_velocity(0);
            }

            if (driver.get_digital(DIGITAL_A) || partner.get_digital(DIGITAL_A)) {
                leftLift.set_value(false);
                rightLift.set_value(false);
            } 
            if (driver.get_digital(DIGITAL_B) || partner.get_digital(DIGITAL_B)) {
                leftLift.set_value(true);
                rightLift.set_value(true);
            }
            
            pros::delay(10);
        }
    } else if (ui.getPartnerIndex() == 1) {
        int blockCount = 0; 
        bool lastBlock = false;

        while (true) {
            int left = driver.get_analog(ANALOG_LEFT_Y);
            int right = driver.get_analog(ANALOG_RIGHT_Y);
            chassis.tank(left, right);

            if (driver.get_digital(DIGITAL_L1) || driver.get_digital(DIGITAL_L2)) {
                intake_mg.move_velocity(-200);

                double torque = intake_mg.get_torque();
                if (torque >= TORQUE_THRESHOLD && !lastBlock) {
                    lastBlock = true;
                    blockCount++;

                    lower_conveyor.move_relative(720, 200); // move lower conveyor

                    // Upper conveyor moves after 4–5 cubes
                    if (blockCount > 4) {
                        upper_conveyor.move_relative(560, 200);
                    }

                    if (blockCount > 6) {
                        driver.rumble("---");
                    }
                } 
                else if (torque < TORQUE_THRESHOLD) {
                    lastBlock = false;
                }
            } 
            else {
                intake_mg.move_velocity(0);
            }

            if (driver.get_digital(DIGITAL_R1)) {
                lower_conveyor.move_velocity(200);
                upper_conveyor.move_velocity(200);
                blockCount = 0;
            }

            else if (driver.get_digital(DIGITAL_R2)) {
                lower_conveyor.move_velocity(-200);
                intake_mg.move_velocity(200);
                blockCount = 0;
            }

            // Stop conveyors if idle
            else if (!(driver.get_digital(DIGITAL_L1) || driver.get_digital(DIGITAL_L2))) {
                lower_conveyor.move_velocity(0);
                upper_conveyor.move_velocity(0);
            }

            if (driver.get_digital(DIGITAL_A)) { // extend
                leftLift.set_value(false);
                rightLift.set_value(false);
            }
            if (driver.get_digital(DIGITAL_B)) { // retract
                leftLift.set_value(true);
                rightLift.set_value(true);
            }

            pros::delay(10);
        }
    }
}