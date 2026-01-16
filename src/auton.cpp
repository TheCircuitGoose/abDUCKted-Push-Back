#include "main.h"
#include "project/auton.hpp"
#include <chrono>

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

trigonometricPositioningSystem::trigonometricPositioningSystem(
    pros::Distance& front, 
    pros::Distance& left, 
    pros::Distance& right, 
    pros::Distance& back, 
    pros::IMU& inertial, 
    float fOffsetX, float fOffsetY, 
    float lOffsetX, float lOffsetY, 
    float rOffsetX, float rOffsetY, 
    float bOffsetX, float bOffsetY
) :
    frontSensor(front),
    leftSensor(left),
    rightSensor(right),
    backSensor(back),
    imuSensor(inertial),
    fOffsetX(fOffsetX), fOffsetY(fOffsetY),
    lOffsetX(lOffsetX), lOffsetY(lOffsetY),
    rOffsetX(rOffsetX), rOffsetY(rOffsetY),
    bOffsetX(bOffsetX), bOffsetY(bOffsetY) {
        rawFrontDistance = 0;
        rawLeftDistance = 0;
        rawRightDistance = 0;
        rawBackDistance = 0;
        imuHeading = 0;
        actualFrontDistance = 0;
        actualLeftDistance = 0;
        actualRightDistance = 0;
        actualBackDistance = 0;
        frontIsValid = false;
        leftIsValid = false;
        rightIsValid = false;
        backIsValid = false;
        position = {0, 0, 0};
}

Pose2D trigonometricPositioningSystem::getLivePosition() {
    return position;
}

void trigonometricPositioningSystem::verifyDistances() {
    // to be implemented
}

void trigonometricPositioningSystem::rotateDistances() {
    // to be implemented
}

void trigonometricPositioningSystem::getPosition() {
    // to be implemented
}

