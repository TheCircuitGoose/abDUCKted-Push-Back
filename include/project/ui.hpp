#pragma once

#include "main.h"
#include "liblvgl/lvgl.h"
#include "project/auton.hpp"

class UI {
    public:
        UI();

        void initUI_NEW(lv_obj_t* parent);

        void initUI(lv_obj_t* parent);

        int getAutonIndex() {
            return lv_roller_get_selected(autonRoller);
        }
        int getColorIndex() {
            return lv_roller_get_selected(colorRoller);
        }
        int getDriveIndex() {
            return lv_roller_get_selected(driveRoller);
        }
        int getPartnerIndex() {
            return lv_roller_get_selected(partnerRoller);
        }
    private:
        lv_obj_t* activeScreen;		// Creates activeScreen parent object

        lv_obj_t* tabview;
        lv_obj_t* tab_home;
        lv_obj_t* tab_auton;
        lv_obj_t* tab_teleop;

        lv_obj_t* autonRoller;		// Creates a roller object as a child of the activeScreen parent
        lv_obj_t* colorRoller;      // Creates a roller object as a child of the activeScreen parent
        lv_obj_t* driveRoller;
        lv_obj_t* partnerRoller;

        lv_obj_t* battery_label;

        int autonIndex = 0;								// Declares an int for storing the selected
        int colorIndex = 0;								// Declares an int for storing the selected color.
        int driveIndex = 0;
        int partnerIndex = 0;

        static void battery_task();

        static void color_roller_event_handler(lv_event_t* e);
};