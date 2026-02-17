#pragma once

#include "main.h"
#include "liblvgl/lvgl.h"
#include "project/auton.hpp"
#include <array>
#include <string>

class UI {
    public:
        UI(); // constructor

        void initUI_NEW(lv_obj_t* parent); // init
        
        int getAutonIndex(); // getters for selected values
        int getColorIndex();

        void goHome();
        void goAuton();
        void goField();

        void updateFieldPose(const Pose& pose);
    private:
        lv_obj_t* activeScreen;		// Creates activeScreen parent object

        lv_obj_t* tabview;          // Tabs
        lv_obj_t* tab_home;
        lv_obj_t* tab_field;
        lv_obj_t* tab_auton;

        lv_obj_t* autonRoller;		// Rollers
        lv_obj_t* colorRoller;

        lv_obj_t* field_container;
        lv_obj_t* field_dot;
        lv_obj_t* field_log_label;

        lv_obj_t* team_label;           // Team number label
        lv_obj_t* battery_container;    // Container for battery display
        lv_obj_t* battery_percent_label; // Battery percentage text
        lv_obj_t* battery_bar;          // Green battery bar

        int autonIndex = 0;			// Declares an int for storing the selected.
        int colorIndex = 0;			// Declares an int for storing the selected color.

        std::array<Pose, 10> pose_log;
        int pose_log_count = 0;
        int pose_log_head = 0;

        static void battery_task(lv_timer_t* timer); // function to update battery percentage on home screen

        static void color_roller_event_handler(lv_event_t* e); // change color of color roller
};