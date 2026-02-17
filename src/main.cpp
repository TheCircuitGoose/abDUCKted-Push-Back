// Include Libraries
#include "main.h"

#include <thread>
#include <string>

#include "liblvgl/lvgl.h"
#include "lemlib/api.hpp"

#include "project/auton.hpp"
#include "project/ui.hpp"

#define TESTING 000 // Encode UI Values for repeated testing, 0 for disable.

#define CONVEYOR_TORQUE_THRESHOLD 3.0

// Device Declarations
// Ports 5, 6, 9, and 13 are Dead =(
pros::Controller driver(pros::E_CONTROLLER_MASTER);				// Creates primary controller
pros::Controller partner(pros::E_CONTROLLER_PARTNER);           // Creates secondary controller

pros::Motor left1(-1, pros::MotorGearset::blue);                // individual motors for logging
pros::Motor left2(-2, pros::MotorGearset::blue);
pros::Motor left3(3, pros::MotorGearset::blue);
pros::Motor right1(4, pros::MotorGearset::blue);
pros::Motor right2(5, pros::MotorGearset::blue);
pros::Motor right3(-6, pros::MotorGearset::blue);

pros::MotorGroup left_mg({-1, -2, 3}, pros::MotorGearset::blue);	// Creates left drive motor group with ports 1, 2, and 3
pros::MotorGroup right_mg({4, 5, -6}, pros::MotorGearset::blue);	// Creates right drive motor group with ports 4, 7, and 8

pros::MotorGroup intake_mg({-7, 8});	                            // Creates intake motor group with ports 10 and 14
pros::MotorGroup conveyor_mg({9, 10});                           // Conveyor motors oppose each other physically, reverse 16 to avoid fighting
pros::Motor lower_conveyor(9);                                    // Creates lower conveyor motor on port 15
pros::Motor upper_conveyor(10);                                    // Creates upper conveyor motor on port 16

pros::ADIDigitalOut descore('H');

pros::Imu inertial(11);												// Creates inertial sensor on port 11
pros::Rotation hTrack(12);											// Creates horizontal tracking wheel on port 11
pros::Rotation vTrack(-13);                                         // Creates vertical tracking wheel on port 17

pros::Distance FL(14);
pros::Distance FR(20);
pros::Distance L(15);
pros::Distance R(19);

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
                                      lemlib::Omniwheel::NEW_275, // AS 2.75" Omni Wheel
                                      0 // Distance from robot center in inches (Negative for Left Side)
);

// Odometry Sensors Configuration
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to vertical rotation sensor.
                            nullptr, // vertical tracking wheel 2, set to null
                            nullptr, // horizontal tracking wheel 1, set to horizontal rotation sensor. 
                            nullptr, // horizontal tracking wheel 2, set to null
                            &inertial // inertial sensor, set to inertial sensor device
);

// Work in Progess
// Lateral PID Controller Configuration
lemlib::ControllerSettings lateral_controller(6.2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              8, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

//Angular PID Controller Configuration
lemlib::ControllerSettings angular_controller(0.85, // proportional gain (kP)
                                              0.004, // integral gain (kI)
                                              0.7, // derivative gain (kD)
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

OdomCorrector odom_corrector(
    FL, 0, 0,
    FR, 0, 0,
    L, 0, 0,
    R, 0, 0,
    144, 144
);

bool isLogging = false;
int targetDistance = 0;

UI ui;

pros::Task* fieldTaskObj = nullptr;

void fieldTask() {
    while (true) {
        auto pose = chassis.getPose();
        Pose current_pose{pose.x, pose.y, pose.theta};
        //Pose corrected_pose = odom_corrector.correct(current_pose);
        ui.updateFieldPose(current_pose);
        pros::delay(50);
    }
}

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

bool disabledThing = 0;
bool inMatch = 0;

// UI Declarations
int autonIndex = 0;	// Declares an int for storing the selected auton routine.
int colorIndex = 0; // Declares an int for storing the selected color.

// When Start
void initialize() {
    lv_init(); // init lvgl
    ui.initUI_NEW(lv_scr_act());

    inertial.reset(); // Reset the inertial sensor
    hTrack.reset(); // Reset the horizontal tracking wheel
    vTrack.reset(); // Reset the vertical tracking wheel
    chassis.calibrate(); // Calibrate the chassis sensors
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    descore.set_value(false);

    pros::Task logTaskObj(logTask); // start logging task
    fieldTaskObj = new pros::Task(fieldTask); // start field update task
}

// When Disabled
void disabled() {}

// When Connected to Field Control
void competition_initialize() {
    ui.goAuton();
    descore.set_value(false);
    inMatch = 1;
}

// When Autonomous
void autonomous() {
    autonIndex = ui.getAutonIndex(); // get selected ui values
    colorIndex = ui.getColorIndex();
    inMatch = 1;
    if ((TESTING / 100) == 1) { // if testing auton
        autonIndex = (TESTING / 10) % 10;
        colorIndex = TESTING % 10;
    }
    ui.goField();
    switch (autonIndex) { // pick auton to use
        case 1: // Left
        {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            chassis.setPose(-50, 17.25, 0); // set starting position, touching parking zone
            pros::delay(10);
            chassis.moveToPoint(-50, 48, 1750, {.maxSpeed = 64}); // drive to match loader
            chassis.turnToHeading(270, 500, {.maxSpeed = 64});
            pros::delay(750);
            chassis.moveToPoint(-64, 48, 1500, {.maxSpeed = 112});
            intake_mg.move_velocity(-200); //unload
            conveyor_mg.move_velocity(-200);
            pros::delay(1250);
            intake_mg.move_velocity(0);
            conveyor_mg.move_velocity(0);
            pros::delay(100);
            chassis.moveToPoint(-25, 48, 2000, {.forwards = false, .maxSpeed = 64}); // go to long goal
            pros::delay(650);
            intake_mg.move_velocity(-200);
            conveyor_anti_jam(lower_conveyor, upper_conveyor, CONVEYOR_TORQUE_THRESHOLD, 3000); // score
            pros::delay(3000);
            chassis.cancelAllMotions();
          break;
        }
        case 2: // Right, mirrored from Left
        {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            chassis.setPose(-50, -17.25, 180); // mirrored start
            pros::delay(10);
            chassis.moveToPoint(-50, -48, 1750, {.maxSpeed = 64}); 
            chassis.turnToHeading(270, 500, {.maxSpeed = 64});
            pros::delay(750);
            chassis.moveToPoint(-64, -48, 1500, {.maxSpeed = 112});
            intake_mg.move_velocity(-200); 
            conveyor_mg.move_velocity(-200);
            pros::delay(1250);
            intake_mg.move_velocity(0);
            conveyor_mg.move_velocity(0);
            pros::delay(100);
            chassis.moveToPoint(-25, -48, 2000, {.forwards = false, .maxSpeed = 64}); 
            pros::delay(650);
            intake_mg.move_velocity(-200);
            conveyor_anti_jam(lower_conveyor, upper_conveyor, CONVEYOR_TORQUE_THRESHOLD, 3000); // score
            pros::delay(3000);
            chassis.cancelAllMotions();
            break;
        }
        case 3: // Right Only Skills
        {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            chassis.setPose(-50, -17.25, 180); // mirrored start
            pros::delay(10);
            chassis.moveToPoint(-50, -48, 1750, {.maxSpeed = 64}); 
            chassis.turnToHeading(270, 500, {.maxSpeed = 64});
            pros::delay(750);
            chassis.moveToPoint(-64, -48, 1500, {.maxSpeed = 112});
            intake_mg.move_velocity(-200); 
            conveyor_mg.move_velocity(-200);
            pros::delay(1250);
            intake_mg.move_velocity(0);
            conveyor_mg.move_velocity(0);
            pros::delay(100);
            chassis.moveToPoint(-25, -48, 2000, {.forwards = false, .maxSpeed = 64}); 
            pros::delay(650);
            intake_mg.move_velocity(-200);
            conveyor_anti_jam(lower_conveyor, upper_conveyor, CONVEYOR_TORQUE_THRESHOLD, 3000); // score
            pros::delay(3000);
            chassis.moveToPoint(-50, -48, 1750, {.maxSpeed = 64}); 
            pros::delay(250);
            chassis.moveToPoint(-64, -48, 1500, {.maxSpeed = 112});
            intake_mg.move_velocity(-200); 
            conveyor_mg.move_velocity(-200);
            pros::delay(1250);
            intake_mg.move_velocity(0);
            conveyor_mg.move_velocity(0);
            pros::delay(100);
            chassis.moveToPoint(-25, -48, 2000, {.forwards = false, .maxSpeed = 64}); 
            pros::delay(650);
            intake_mg.move_velocity(-200);
            conveyor_anti_jam(lower_conveyor, upper_conveyor, CONVEYOR_TORQUE_THRESHOLD, 3000);
            pros::delay(1000);

            chassis.moveToPoint(-36, -48, 1750, {.maxSpeed = 64}); 
            chassis.moveToPoint(-48, 48, 2000, {.maxSpeed = 64});
            chassis.moveToPoint(-24, 16, 3000, {.maxSpeed = 64});
            chassis.turnToHeading(75, 1500, {.maxSpeed = 64});
            chassis.moveToPoint(-64, 0, 20000, {.forwards = false});
            pros::delay(5000);
            break;
        }
        case 4: // Just Move
        {
            upper_conveyor.move_velocity(600);
            pros::delay(1000);
            upper_conveyor.move_velocity(0);
            break;
        }
        case 5: // Simple Forward
        {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            pros::delay(1000);
            chassis.setPose(0, 0, 0);
            chassis.moveToPose(0, 5, 0, 10000);
            break;
        }
        case 6: // Lat PID Tuning
        {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            pros::delay(4750);
            chassis.setPose(0, 0, 0);
            isLogging = true; // Start logging data
            pros::delay(250);
            targetDistance = 24;
            chassis.moveToPose(0, 24, 0, 10000);
            break;
        }
        case 7: // Ang PID Tuning
        {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            pros::delay(4750);
            chassis.setPose(0, 0, 0);
            isLogging = true; // Start logging data
            pros::delay(250);
            targetDistance = 90;
            chassis.turnToHeading(90, 10000);
            break;
        }
        default:
        {
            break;
        }
    }
}

// When Driver Control
void opcontrol() {
    disabledThing = 0;
    if ((TESTING / 100) == 1) { // if testing auton
        autonomous();
    }
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST); // Set the brake mode to brake
    ui.goHome();
    descore.set_value(false);
    while (true) {
            int left = driver.get_analog(ANALOG_LEFT_Y); // Gets Left Stick Up/Down Value
            int right = driver.get_analog(ANALOG_RIGHT_Y); // Gets Right Stick Up/Down Value
            chassis.tank(left, right);

            if (driver.get_digital(DIGITAL_L1)) { // intake
                intake_mg.move_velocity(-200);
            } else if (driver.get_digital(DIGITAL_L2)) {
                intake_mg.move_velocity(200);
            } else {
                intake_mg.move_velocity(0);
            }

            if (driver.get_digital(DIGITAL_R1)) { // conveyor
                conveyor_mg.move_velocity(200);
            } else if (driver.get_digital(DIGITAL_R2)) {
                conveyor_mg.move_velocity(-200);
            } else {
                conveyor_mg.move_velocity(0);
            }

            if (driver.get_digital(DIGITAL_Y) || disabledThing) {
                descore.set_value(false);
            } else {
                descore.set_value(true);
            }

            if (driver.get_digital(DIGITAL_DOWN) && !inMatch) {
                if (disabledThing == 0) {
                    disabledThing = 1;
                    pros::delay(1000);
                } else {
                    pros::delay(1000);
                    disabledThing = 0;
                }
            }
            pros::delay(10);
    }
}