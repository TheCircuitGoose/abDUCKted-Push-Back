#pragma once                                               // Library is only called once

#include <string>                                          // Include string library

namespace auton {                                          // Creates "auton" namespace
    const std::string autonNames =                         // List of autons to be selected for display
        "Disabled\nLeft LG\nRight LG\nLoader Skills\nParking Skills\nDrive\nLateral PID Tuning\nAngular PID Tuning";                           
}

namespace color {
    const std::string colorNames =                         // List of colors to be selected for display
        "Timer\nColor Sensing (Red)\nColor Sensing (Blue)\nTorque Sensing\nBlanking Sensing";
}

int get_color(pros::Optical& intake_color);
void wait_for_intake_torque(pros::MotorGroup& intake_mg, int block_quantity, float torque_threshold, int timeout);
void wait_for_intake_color(pros::MotorGroup& intake_mg, pros::Optical& intake_color, int color_index);
void wait_for_blank(pros::MotorGroup& intake_mg, pros::Optical& loader_color, int timeout);