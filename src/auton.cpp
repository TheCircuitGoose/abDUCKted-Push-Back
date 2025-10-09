#include "main.h"

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