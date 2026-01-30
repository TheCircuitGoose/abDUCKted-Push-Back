#pragma once                                               // Library is only called once

#include <string>                                          // Include string library
#include <array>

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

class trigonometricPositioningSystem {
    public:
        trigonometricPositioningSystem(pros::Distance& front, 
                                       pros::Distance& left, 
                                       pros::Distance& right, 
                                       pros::Distance& back, 
                                       pros::IMU& inertial, 
                                       float fOffsetX, float fOffsetY, 
                                       float lOffsetX, float lOffsetY, 
                                       float rOffsetX, float rOffsetY, 
                                       float bOffsetX, float bOffsetY);

        Pose2D getLivePosition();
        std::array<float, 3> getPositionArray();
    private:
        pros::Distance& frontSensor;
        pros::Distance& leftSensor;
        pros::Distance& rightSensor;
        pros::Distance& backSensor;
        pros::IMU& imuSensor;
        float fOffsetX;
        float fOffsetY;
        float lOffsetX;
        float lOffsetY;
        float rOffsetX;
        float rOffsetY;
        float bOffsetX;
        float bOffsetY;

        float rawFrontDistance;
        float rawLeftDistance;
        float rawRightDistance;
        float rawBackDistance;
        float imuHeading;

        float actualFrontDistance;
        float actualLeftDistance;
        float actualRightDistance;
        float actualBackDistance;

        bool frontIsValid;
        bool leftIsValid;
        bool rightIsValid;
        bool backIsValid;

        Pose2D position;

        void verifyDistances();
        void rotateDistances();
        void getPosition();
};