#pragma once                                               // Library is only called once

#include <string>                                          // Include string library
#include <array>
#include "main.h"

typedef struct {
    float x;
    float y;
    float heading;
} Pose2D;

namespace auton {                                          // Creates "auton" namespace
    const std::string autonNames =                         // List of autons to be selected for display
        "Disabled\nLeft LG\nRight LG\nLoader Skills\nParking Skills\nDrive\nLateral PID Tuning\nAngular PID Tuning";                           
}

namespace color {
    const std::string colorNames =                         // List of colors to be selected for display
        "Timer\nBlanking Sensing";
}

class Timer {
    public:
        void start();
        void stop();
        long getTime();
    private:
        std::chrono::high_resolution_clock::time_point startTime;
        std::chrono::high_resolution_clock::time_point endTime;
};

void wait_for_blank(pros::MotorGroup& intake_mg, pros::Optical& loader_color, int timeout);

struct Pose {
    double x;
    double y;
    double theta;
};

class OdomCorrector {
    private:
        pros::Distance& FL;
        pros::Distance& FR;
        pros::Distance& L;
        pros::Distance& R;

        double fl_x, fl_y;
        double fr_x, fr_y;
        double l_x, l_y;
        double r_x, r_y;

        double field_width;
        double field_height;

        const double MAX_DISTANCE = 48.0;
        const double MAX_CORRECTION = 6.0;
        const double BLEND = 0.25;

    public:
        OdomCorrector(
            pros::Distance& FL, double fl_x, double fl_y,
            pros::Distance& FR, double fr_x, double fr_y,
            pros::Distance& L, double l_x, double l_y,
            pros::Distance& R, double r_x, double r_y,
            double field_width, double field_height
        );

        Pose correct(Pose current_pose);
};

