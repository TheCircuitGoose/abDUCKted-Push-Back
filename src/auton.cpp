#include "main.h"
#include "project/auton.hpp"
#include <chrono>
#include <cmath>

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

std::array<float, 3> trigonometricPositioningSystem::getPositionArray() {
    getPosition();
    return {position.x, position.y, position.heading};
}

void trigonometricPositioningSystem::verifyDistances() {
    const float fieldSize = 144.0f; // 12 ft field in inches
    const float diagSize = fieldSize * 1.41421356237f; // diagonal across the field
    const float pairTolerance = 3.0f; // allowable error for opposite wall sums
    const float diagTolerance = 4.0f; // allowable error for diagonal checks

    auto inRange = [fieldSize](float distance) {
        return distance > 0.5f && distance < fieldSize;
    };

    frontIsValid = inRange(actualFrontDistance);
    backIsValid = inRange(actualBackDistance);
    leftIsValid = inRange(actualLeftDistance);
    rightIsValid = inRange(actualRightDistance);

    if (frontIsValid && backIsValid) {
        float sumFB = actualFrontDistance + actualBackDistance;
        if (std::fabs(sumFB - fieldSize) > pairTolerance) {
            frontIsValid = false;
            backIsValid = false;
        }
    }

    if (leftIsValid && rightIsValid) {
        float sumLR = actualLeftDistance + actualRightDistance;
        if (std::fabs(sumLR - fieldSize) > pairTolerance) {
            leftIsValid = false;
            rightIsValid = false;
        }
    }

    if (frontIsValid && backIsValid && leftIsValid && rightIsValid) {
        float diagA = std::hypot(actualFrontDistance, actualRightDistance) + std::hypot(actualBackDistance, actualLeftDistance);
        float diagB = std::hypot(actualFrontDistance, actualLeftDistance) + std::hypot(actualBackDistance, actualRightDistance);
        bool diagOk = (std::fabs(diagA - diagSize) <= diagTolerance) || (std::fabs(diagB - diagSize) <= diagTolerance);
        if (!diagOk) {
            frontIsValid = false;
            backIsValid = false;
            leftIsValid = false;
            rightIsValid = false;
        }
    }
}

void trigonometricPositioningSystem::rotateDistances() {
    constexpr float degToRad = 3.14159265358979323846f / 180.0f;
    float headingRad = imuHeading * degToRad;
    float sinH = std::sin(headingRad);
    float cosH = std::cos(headingRad);

    // Robot forward/right vectors expressed in field frame (0 deg faces +Y/north)
    float forwardX = sinH;
    float forwardY = cosH;
    float rightX = cosH;
    float rightY = -sinH;

    auto project = [](float vx, float vy, float dirX, float dirY) {
        return vx * dirX + vy * dirY;
    };

    actualFrontDistance = rawFrontDistance + project(fOffsetX, fOffsetY, forwardX, forwardY);
    actualBackDistance  = rawBackDistance  + project(bOffsetX, bOffsetY, -forwardX, -forwardY);
    actualLeftDistance  = rawLeftDistance  + project(lOffsetX, lOffsetY, -rightX, -rightY);
    actualRightDistance = rawRightDistance + project(rOffsetX, rOffsetY, rightX, rightY);
}

void trigonometricPositioningSystem::getPosition() {
    // Read raw sensor values (Distance sensor returns millimeters)
    rawFrontDistance = frontSensor.get() / 25.4f;
    rawLeftDistance = leftSensor.get() / 25.4f;
    rawRightDistance = rightSensor.get() / 25.4f;
    rawBackDistance = backSensor.get() / 25.4f;
    imuHeading = imuSensor.get_heading();

    rotateDistances();
    verifyDistances();

    const float halfField = 72.0f; // half of 12 ft field in inches

    float x;
    float y;

    if (leftIsValid && rightIsValid) {
        x = (actualLeftDistance - actualRightDistance) / 2.0f;
    } else if (rightIsValid) {
        x = halfField - actualRightDistance;
    } else if (leftIsValid) {
        x = actualLeftDistance - halfField;
    } else {
        x = 999999.0f; // both invalid → failure marker
    }

    if (frontIsValid && backIsValid) {
        y = (actualBackDistance - actualFrontDistance) / 2.0f;
    } else if (frontIsValid) {
        y = halfField - actualFrontDistance;
    } else if (backIsValid) {
        y = actualBackDistance - halfField;
    } else {
        y = 999999.0f; // both invalid → failure marker
    }

    // Heading is maintained in degrees from the IMU, where 0 deg faces north.
    position = {x, y, imuHeading};
}

