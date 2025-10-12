#pragma once                                               // Library is only called once

#include <string>                                          // Include string library

namespace auton {                                          // Creates "auton" namespace
    const std::string autonNames =                         // List of autons to be selected for display
        "Disabled\nLeft\nRight\nPID Tuning";                           
}

namespace color {
    const std::string colorNames =                         // List of colors to be selected for display
        "Simple Timer\nColor Sensing (Red)\nColor Sensing (Blue)\nTorque Sensing";
}

int get_color(pros::Optical& intake_color);

void wait_for_intake(pros::MotorGroup& intake_mg, int block_quantity, float torque_threshold);