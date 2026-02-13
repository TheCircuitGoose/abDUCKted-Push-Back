#include "main.h"
#include "project/auton.hpp"
#include <chrono>
#include <cmath>

static constexpr double MM_TO_IN = 1.0 / 25.4;

using namespace std;

void Timer::start() {
    startTime = chrono::high_resolution_clock::now();
}
void Timer::stop() {
    endTime = chrono::high_resolution_clock::now();
}
long Timer::getTime() {
    return chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
}

void wait_for_blank(pros::MotorGroup& intake_mg, pros::Optical& loader_color, int timeout) {
    intake_mg.move_velocity(200);
    while ((loader_color.get_hue() > 180 && loader_color.get_hue() < 230) ||
           (loader_color.get_hue() > 345) || (loader_color.get_hue() < 15)) {
            pros::delay(0.1); // 10khz loop for most precise timing
    }
    intake_mg.move_velocity(0);
    return;
}

OdomCorrector::OdomCorrector(
    pros::Distance& FL, double fl_x, double fl_y,
    pros::Distance& FR, double fr_x, double fr_y,
    pros::Distance& L,  double l_x,  double l_y,
    pros::Distance& R,  double r_x,  double r_y,
    double field_width, double field_height
)
: FL(FL), FR(FR), L(L), R(R),
  fl_x(fl_x), fl_y(fl_y),
  fr_x(fr_x), fr_y(fr_y),
  l_x(l_x),   l_y(l_y),
  r_x(r_x),   r_y(r_y),
  field_width(field_width),
  field_height(field_height)
{}

Pose OdomCorrector::correct(Pose current_pose) {
    double new_x = current_pose.x;
    double new_y = current_pose.y;
    double new_theta = current_pose.theta;

    double fl_read = FL.get() * MM_TO_IN;
    double fr_read = FR.get() * MM_TO_IN;
    double l_read  = L.get()  * MM_TO_IN;
    double r_read  = R.get()  * MM_TO_IN;

    if (fl_read < MAX_DISTANCE) {
        if (fr_read < MAX_DISTANCE) {
            double sensor_spacing = fl_x - fr_x;
            if (std::fabs(sensor_spacing) > 1e-6) {
                double theta_front = std::atan((fl_read - fr_read) / sensor_spacing);
                double avg_front = (fl_read + fr_read) / 2.0;
                double predicted_y = (field_height / 2.0) - avg_front - fl_y;
                if (std::fabs(predicted_y - current_pose.y) < MAX_CORRECTION) {
                    new_y = current_pose.y * (1.0 - BLEND) + predicted_y * BLEND;
                    new_theta = current_pose.theta * (1.0 - BLEND) + theta_front * BLEND;
                }
            }
        }
    }

    if (l_read < MAX_DISTANCE) {
        double predicted_x = (-field_width / 2.0) + l_read + l_x;
        if (std::fabs(predicted_x - current_pose.x) < MAX_CORRECTION) {
            new_x = current_pose.x * (1.0 - BLEND) + predicted_x * BLEND;
        }
    }

    if (r_read < MAX_DISTANCE) {
        double predicted_x = (field_width / 2.0) - r_read - r_x;
        if (std::fabs(predicted_x - current_pose.x) < MAX_CORRECTION) {
            new_x = current_pose.x * (1.0 - BLEND) + predicted_x * BLEND;
        }
    }

    if (new_x < -field_width / 2.0) { return current_pose; }
    if (new_x >  field_width / 2.0) { return current_pose; }
    if (new_y < -field_height / 2.0) { return current_pose; }
    if (new_y >  field_height / 2.0) { return current_pose; }

    Pose corrected_pose;
    corrected_pose.x = new_x;
    corrected_pose.y = new_y;
    corrected_pose.theta = new_theta;

    return corrected_pose;
}