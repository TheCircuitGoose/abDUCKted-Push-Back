#include "main.h"
#include "project/auton.hpp"
#include <chrono>

using namespace std;

class Timer {
    public:
        void start() {
            startTime = chrono::high_resolution_clock::now();
        }
        void stop() {
            endTime = chrono::high_resolution_clock::now();
        }
        long getTime() {
            return chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
        }
    private:
        chrono::high_resolution_clock::time_point startTime;
        chrono::high_resolution_clock::time_point endTime;
};

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

void wait_for_intake_torque(pros::MotorGroup& intake_mg, int block_quantity, float torque_threshold, int timeout) {
    bool lastBlock = false;
    int blockCount = 0;
    Timer timer;
    timer.start();
    timer.stop();
    
    while ((blockCount < block_quantity) && (timer.getTime() < timeout)) {
        double torque = intake_mg.get_torque();
        if (torque >= torque_threshold && !lastBlock) { // detect if block was intaken
            lastBlock = true;
            blockCount++;
        } 
        else if (torque < torque_threshold) {
            lastBlock = false; // debounce
        }
        pros::delay(10);
        timer.stop();
    }
    return;
}

void wait_for_intake_color(pros::MotorGroup& intake_mg, pros::Optical& intake_color, int color_index) {
    while (get_color(intake_color) != color_index) { // wait until other color shows if using color sensing
        pros::delay(10);
    }
    intake_mg.move_velocity(200);
    pros::delay(500);
    intake_mg.move_velocity(0);
    return;
}