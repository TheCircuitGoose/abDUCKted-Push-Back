#pragma once

#include "main.h"
#include "liblvgl/lvgl.h"
#include "project/auton.hpp"

class UI {
    public:
        UI(); // constructor

        void initUI_NEW(lv_obj_t* parent); // inits
        void initUI(lv_obj_t* parent);

        int getAutonIndex(); // getters for selected values
        int getColorIndex();
        int getDriveIndex();
        int getPartnerIndex();
    private:
        lv_obj_t* activeScreen;		// Creates activeScreen parent object

        lv_obj_t* tabview;          // Tabs
        lv_obj_t* tab_home;
        lv_obj_t* tab_auton;
        lv_obj_t* tab_teleop;

        lv_obj_t* autonRoller;		// Rollers
        lv_obj_t* colorRoller;
        lv_obj_t* driveRoller;
        lv_obj_t* partnerRoller;

        lv_obj_t* battery_label;

        int autonIndex = 0;			// Declares an int for storing the selected.
        int colorIndex = 0;			// Declares an int for storing the selected color.
        int driveIndex = 0;         // Declares an int for storing the selected drive scheme.
        int partnerIndex = 0;       // Declares an int for storing the selected partner mode.

        static void battery_task(); // function to update battery percentage on home screen

        static void color_roller_event_handler(lv_event_t* e); // change color of color roller
};