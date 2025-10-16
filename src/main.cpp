// Include Libraries
#include "main.h"

#include <thread>
#include <string>

#include "liblvgl/lvgl.h"
#include "lemlib/api.hpp"

#include "project/auton.hpp"
#include "project/ui.hpp"

#define TORQUE_THRESHOLD 0.15
#define TESTING 0 // Encode UI Values for repeated testing, 0 for disable.

// Device Declarations
// Ports 5, 6, 9, and 13 are Dead =(
pros::Controller driver(pros::E_CONTROLLER_MASTER);				// Creates primary controller
pros::Controller partner(pros::E_CONTROLLER_PARTNER);           // Creates secondary controller

pros::Motor left1(-1, pros::MotorGearset::blue);                // individual motors for logging
pros::Motor left2(-2, pros::MotorGearset::blue);
pros::Motor left3(3, pros::MotorGearset::blue);
pros::Motor right1(4, pros::MotorGearset::blue);
pros::Motor right2(7, pros::MotorGearset::blue);
pros::Motor right3(-8, pros::MotorGearset::blue);

pros::MotorGroup left_mg({-1, -2, 3}, pros::MotorGearset::blue);	// Creates left drive motor group with ports 1, 2, and 3
pros::MotorGroup right_mg({4, 7, -8}, pros::MotorGearset::blue);	// Creates right drive motor group with ports 4, 7, and 8

pros::MotorGroup intake_mg({10, -14});	                            // Creates intake motor group with ports 10 and 14
pros::Motor lower_conveyor(-15);                                    // Creates lower conveyor motor on port 15
pros::Motor upper_conveyor(-16);                                    // Creates upper conveyor motor on port 16

pros::ADIDigitalOut leftLift('A');
pros::ADIDigitalOut rightLift('B');

pros::Imu inertial(11);												// Creates inertial sensor on port 11
pros::Rotation hTrack(12);											// Creates horizontal tracking wheel on port 11
pros::Rotation vTrack(-17);                                         // Creates vertical tracking wheel on port 17

pros::Optical intake_color(18);                                     // Creates optical sensor on port 18

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
lemlib::ControllerSettings angular_controller(1.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              8, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
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
int autonIndex = 0;	// Declares an int for storing the selected auton routine.
int colorIndex = 0; // Declares an int for storing the selected color.
UI ui;

// When Start
void initialize() {
    lv_init(); // init lvgl
    ui.initUI_NEW(lv_scr_act());

    inertial.reset(); // Reset the inertial sensor
    hTrack.reset(); // Reset the horizontal tracking wheel
    vTrack.reset(); // Reset the vertical tracking wheel
    chassis.calibrate(); // Calibrate the chassis sensors
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    pros::Task logTaskObj(logTask); // start logging task
}

// When Disabled
void disabled() {}

// When Connected to Field Control
void competition_initialize() {}

// When Autonomous
void autonomous() {
    autonIndex = ui.getAutonIndex(); // get selected ui values
    colorIndex = ui.getColorIndex();
    if ((TESTING / 100) == 1) { // if testing left auton
        autonIndex = (TESTING / 10) % 10;
        colorIndex = TESTING % 10;
    }
    ui.goHome();
    switch (autonIndex) { // pick auton to use
        case 1: // Left
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            chassis.setPose(-50, 18, 0); // set starting position, touching parking zone
            leftLift.set_value(false);
            rightLift.set_value(false);
            pros::delay(10);
            chassis.moveToPoint(-50, 48, 2000); // drive to match loader
            chassis.turnToHeading(270, 750);
            chassis.moveToPoint(-59, 48, 1750, {.maxSpeed = 75});
            intake_mg.move_velocity(-200); //unload
            lower_conveyor.move_velocity(200);
            if (colorIndex == 1 || colorIndex == 2) { // if using color sensing
                while (get_color(intake_color) != colorIndex) { // wait until other color shows if using color sensing
                    pros::delay(10);
                }
                intake_mg.move_velocity(0);
                lower_conveyor.move_velocity(0);
            } else if (colorIndex == 3) { // if using torque sensing
                wait_for_intake(intake_mg, 3, TORQUE_THRESHOLD); // wait for 3 blocks to be picked up
            } else {
                pros::delay(2000); // wait 2 second if not using sensing
                intake_mg.move_velocity(0);
                lower_conveyor.move_velocity(0);
            }
            chassis.moveToPoint(-44, 48, 2000, {.forwards = false}); // go to long goal
            lower_conveyor.move_velocity(0);
            upper_conveyor.move_velocity(0);
            intake_mg.move_velocity(0);
            pros::delay(50);
            chassis.turnToHeading(90, 1250, {.maxSpeed = 125});
            pros::delay(50);
            chassis.moveToPoint(-26, 48, 2500, {.maxSpeed = 125});
            pros::delay(1000);
            lower_conveyor.move_velocity(200); // score
            upper_conveyor.move_velocity(200);
            intake_mg.move_velocity(-200);
            pros::delay(2950);
            chassis.setPose(-32, 48, 90); // Reset position to avoid drift (Instead of using Ki)
            lower_conveyor.move_velocity(0);
            upper_conveyor.move_velocity(0);
            intake_mg.move_velocity(0);
            pros::delay(50);
            chassis.moveToPoint(-48, 48, 2000, {.forwards = false});
            chassis.turnToHeading(135, 750);
            intake_mg.move_velocity(-200);
            chassis.moveToPoint(-36, 36, 1750);
            //chassis.moveToPoint(-22, 26, 3000, {.maxSpeed = 75});
            break;
        case 2: // Right, same as left but mirrored
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            chassis.setPose(-50, -18, 180);
            leftLift.set_value(false);
            rightLift.set_value(false);
            pros::delay(10);
            chassis.moveToPoint(-50, -48, 3000);
            chassis.turnToHeading(270, 1000);
            chassis.moveToPoint(-59, -48, 3000, {.maxSpeed = 75});
            intake_mg.move_velocity(-200);
            lower_conveyor.move_velocity(200);
            if (colorIndex == 1 || colorIndex == 2) { // if using color sensing
                while (get_color(intake_color) != colorIndex) { // wait until other color shows if using color sensing
                    pros::delay(10);
                }
                intake_mg.move_velocity(0);
                lower_conveyor.move_velocity(0);
            } else if (colorIndex == 3) { // if using torque sensing
                wait_for_intake(intake_mg, 3, TORQUE_THRESHOLD); // wait for 3 blocks to be picked up
            } else {
                pros::delay(2000); // wait 2 second if not using sensing
                intake_mg.move_velocity(0);
                lower_conveyor.move_velocity(0);
            }
            chassis.moveToPoint(-44, -48, 2000, {.forwards = false});
            lower_conveyor.move_velocity(0);
            upper_conveyor.move_velocity(0);
            intake_mg.move_velocity(0);
            pros::delay(50);
            chassis.turnToHeading(90, 750);
            pros::delay(50);
            chassis.moveToPose(-26, -48, 90, 3000);
            pros::delay(1000);
            lower_conveyor.move_velocity(200);
            upper_conveyor.move_velocity(200);
            intake_mg.move_velocity(-200);
            pros::delay(2950);
            chassis.setPose(-32, -48, 90);
            lower_conveyor.move_velocity(0);
            upper_conveyor.move_velocity(0);
            intake_mg.move_velocity(0);
            pros::delay(50);
            chassis.moveToPoint(-48, -48, 2000, {.forwards = false});
            chassis.turnToHeading(225, 750);
            intake_mg.move_velocity(-200);
            chassis.moveToPoint(-36, -36, 1750);
            break;
        case 3: // PID Tuning
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            pros::delay(4750);
            chassis.setPose(0, 0, 0);
            isLogging = true; // Start logging data
            pros::delay(250);
            targetDistance = 72;
            chassis.moveToPose(0, 72, 0, 10000);
            break;
        default:
            break;
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

            if (driver.get_digital(DIGITAL_L1) || partner.get_digital(DIGITAL_L1)) { // intake
                intake_mg.move_velocity(200);
            } else if (driver.get_digital(DIGITAL_L2) || partner.get_digital(DIGITAL_L2)) {
                intake_mg.move_velocity(-200);
            } else {
                intake_mg.move_velocity(0);
            }

            if (driver.get_digital(DIGITAL_R1) || partner.get_digital(DIGITAL_R1)) { // conveyor
                lower_conveyor.move_velocity(200);
                upper_conveyor.move_velocity(200);
            } else if (driver.get_digital(DIGITAL_R2) || partner.get_digital(DIGITAL_R2)) {
                lower_conveyor.move_velocity(-200);
                upper_conveyor.move_velocity(-200);
            } else {
                lower_conveyor.move_velocity(0);
                upper_conveyor.move_velocity(0);
            }

            if (partner.get_digital(DIGITAL_UP)) { // lower conveyor (partner only)
                lower_conveyor.move_velocity(-200);
            } else if (partner.get_digital(DIGITAL_DOWN)) {
                lower_conveyor.move_velocity(200);
            } else {
                lower_conveyor.move_velocity(0);
            }

            if (partner.get_digital(DIGITAL_X)) { // upper conveyor (partner only)
                upper_conveyor.move_velocity(-200);
            } else if (partner.get_digital(DIGITAL_B)) {
                upper_conveyor.move_velocity(200);
            } else {
                upper_conveyor.move_velocity(0);
            }

            if (driver.get_digital(DIGITAL_Y)) { // lift
                leftLift.set_value(false);
                rightLift.set_value(false);
            } 
            if (driver.get_digital(DIGITAL_B)) {
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
                intake_mg.move_velocity(-200); // intake

                double torque = intake_mg.get_torque();
                if (torque >= TORQUE_THRESHOLD && !lastBlock) { // detect if block was intaken
                    lastBlock = true;
                    blockCount++;

                    lower_conveyor.move_relative(720, 200); // move lower conveyor

                    if (blockCount > 4) { // move upper conveyor once bottom is full
                        upper_conveyor.move_relative(560, 200);
                    }

                    if (blockCount > 6) { // rumble if full to indicate to driver
                        driver.rumble("-");
                    }
                } 
                else if (torque < TORQUE_THRESHOLD) {
                    lastBlock = false; // debounce
                }
            } 
            else {
                intake_mg.move_velocity(0);
            }

            if (driver.get_digital(DIGITAL_R1)) { // score
                lower_conveyor.move_velocity(200);
                upper_conveyor.move_velocity(200);
                blockCount = 0; // reset count
            }

            else if (driver.get_digital(DIGITAL_R2)) { // reverse score
                lower_conveyor.move_velocity(-200);
                intake_mg.move_velocity(200);
                blockCount = 0; // reset count
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
    } else if (partnerIndex == 2) {
        while (true) {
            int left = driver.get_analog(ANALOG_LEFT_Y); // Gets Left Stick Up/Down Value
            int right = driver.get_analog(ANALOG_RIGHT_Y); // Gets Right Stick Up/Down Value
            chassis.tank(left, right);

            if (driver.get_digital(DIGITAL_L1)) {
                intake_mg.move_velocity(200);
            } else if (driver.get_digital(DIGITAL_L2)) {
                intake_mg.move_velocity(-200);
            } else {
                intake_mg.move_velocity(0);
            }

            if (driver.get_digital(DIGITAL_R1)) {
                lower_conveyor.move_velocity(200);
                upper_conveyor.move_velocity(200);
            } else if (driver.get_digital(DIGITAL_R2)) {
                lower_conveyor.move_velocity(-200);
                upper_conveyor.move_velocity(-200);
            } else {
                lower_conveyor.move_velocity(0);
                upper_conveyor.move_velocity(0);
            }

            if (driver.get_digital(DIGITAL_Y)) {
                leftLift.set_value(false);
                rightLift.set_value(false);
            } 
            if (driver.get_digital(DIGITAL_B)) {
                leftLift.set_value(true);
                rightLift.set_value(true);
            }
            
            pros::delay(10);
        }
    }
}