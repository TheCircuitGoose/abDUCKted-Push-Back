#pragma once

#include "main.h"
#include "liblvgl/lvgl.h"
#include "project/auton.hpp"

class UI {
    public:
        int getAutonIndex() {
            return lv_roller_get_selected(autonRoller);
        }
        int getColorIndex() {
            return lv_roller_get_selected(colorRoller);
        }

        void initUI(lv_obj_t* parent) {
            activeScreen = lv_obj_create(parent);
            autonRoller = lv_roller_create(activeScreen);
            colorRoller = lv_roller_create(activeScreen);

            lv_obj_set_style_text_font(										// Set font size to 36 pt.
            activeScreen, 
            &lv_font_montserrat_18, 
            LV_PART_MAIN | LV_STATE_DEFAULT
        );
        // Configure Auton Roller
            lv_obj_set_size(activeScreen, 480, 220);						// Configure size & position of activeScreen Parent
            lv_obj_center(activeScreen);
            lv_roller_set_options(											// Configure Roller
                autonRoller, 
                auton::autonNames.c_str(), 
                LV_ROLLER_MODE_NORMAL
            );
            lv_roller_set_visible_row_count(autonRoller, 4);
            lv_obj_set_style_bg_color(										// Set highlight color of selected choice to a bold yellow
                autonRoller, 
                lv_color_hex(0xFFFF00), 
                LV_PART_SELECTED
            );
            lv_obj_set_style_text_color(									// Set text color to black
                autonRoller, 
                lv_color_hex(0x000000), 
                LV_PART_SELECTED
            );
            lv_obj_set_size(autonRoller, 238, 220);							// Configure size & position of roller object
            lv_obj_align(autonRoller, LV_ALIGN_LEFT_MID, 0, 0);

            // Configure Color Roller
            lv_roller_set_options(
                colorRoller,
                color::colorNames.c_str(),
                LV_ROLLER_MODE_NORMAL
            );
            lv_roller_set_visible_row_count(colorRoller, 2);
            lv_obj_set_style_bg_color(
                colorRoller,
                lv_color_hex(0xFFFF00),
                LV_PART_SELECTED
            );
            lv_obj_set_style_text_color(
                colorRoller,
                lv_color_hex(0x000000),
                LV_PART_SELECTED
            );
            lv_obj_set_size(colorRoller, 238, 220);						// Configure size & position of roller object
            lv_obj_align(colorRoller, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_obj_add_event_cb(colorRoller, color_roller_event_handler, LV_EVENT_ALL, NULL);
            };
    private:
        lv_obj_t* activeScreen;		// Creates activeScreen parent object
        lv_obj_t* autonRoller;		// Creates a roller object as a child of the activeScreen parent
        lv_obj_t* colorRoller;      // Creates a roller object as a child of the activeScreen parent

        int autonIndex = 0;								// Declares an int for storing the selected
        int colorIndex = 0;								// Declares an int for storing the selected color.

        static void color_roller_event_handler(lv_event_t * e) { // change color of color selector
            lv_event_code_t code = lv_event_get_code(e);
            lv_obj_t * roller = static_cast<lv_obj_t *>(lv_event_get_target(e));

            if (code == LV_EVENT_VALUE_CHANGED) {
                int selected = lv_roller_get_selected(roller);
                if (selected == 1) { // Red
                    lv_obj_set_style_bg_color(roller, lv_color_hex(0xFF0000), LV_PART_SELECTED);
                } else if (selected == 2) { // Blue
                    lv_obj_set_style_bg_color(roller, lv_color_hex(0x0000FF), LV_PART_SELECTED);
                } else {
                    lv_obj_set_style_bg_color(roller, lv_color_hex(0xFFFF00), LV_PART_SELECTED);
                };
            };
        };
};