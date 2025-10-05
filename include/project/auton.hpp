#pragma once                                               // Library is only called once

#include <string>                                          // Include string library

namespace auton {                                          // Creates "auton" namespace
    const std::string autonNames =                         // List of autons to be selected for display
        "Disabled\nLeft\nRight\nPID Tuning";                           
}

namespace color {
    const std::string colorNames =                         // List of colors to be selected for display
        "Disable Color Sensing\nRed Alliance\nBlue Alliance";
}

int get_color(pros::Optical& intake_color) {               // Color sensing wrapper function for auton
    int hue = intake_color.get_hue();
    int color = 0; // Default to no color

    if ((hue >= 0 && hue <= 20) || (hue >= 350 && hue <= 360)) {
        color = 1; // Red
    } else if (hue >= 195 && hue <= 240) {
        color = 2; // Blue
    }

    return color;
}