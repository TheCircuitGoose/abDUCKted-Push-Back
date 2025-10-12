#include "main.h"
#include "liblvgl/lvgl.h"

#include "project/ui.hpp"

UI::UI() {};

int UI::getAutonIndex() { // getters for selected values
    return lv_roller_get_selected(autonRoller); // return the index of roller
}
int UI::getColorIndex() {
    return lv_roller_get_selected(colorRoller);
}
int UI::getDriveIndex() {
    return lv_roller_get_selected(driveRoller);
}
int UI::getPartnerIndex() {
    return lv_roller_get_selected(partnerRoller);
}

void UI::initUI_NEW(lv_obj_t* parent) {
    tabview = lv_tabview_create(parent, LV_DIR_TOP, 40); // tabs
    tab_home = lv_tabview_add_tab(tabview, "Home");
    tab_auton = lv_tabview_add_tab(tabview, "Auton");
    tab_teleop = lv_tabview_add_tab(tabview, "Driver");

    autonRoller = lv_roller_create(tab_auton); // rollers
    colorRoller = lv_roller_create(tab_auton);
    driveRoller = lv_roller_create(tab_teleop);
    partnerRoller = lv_roller_create(tab_teleop);

    // AUTON TAB
    lv_obj_set_size(tab_auton, 480, 220);
    lv_obj_center(tab_auton);

    // Configure Auton Roller
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

    // DRIVE TAB
    lv_obj_set_size(tab_teleop, 480, 220);
    lv_obj_center(tab_teleop);

    // Configure Drive Roller
    lv_roller_set_options(											// Configure Roller
        driveRoller, 
        "Tank\nArcade", 
        LV_ROLLER_MODE_NORMAL
    );
    lv_roller_set_visible_row_count(driveRoller, 4);
    lv_obj_set_style_bg_color(										// Set highlight color of selected choice to a bold yellow
        driveRoller, 
        lv_color_hex(0xFFFF00), 
        LV_PART_SELECTED
    );
    lv_obj_set_style_text_color(									// Set text color to black
        driveRoller, 
        lv_color_hex(0x000000), 
        LV_PART_SELECTED
    );
    lv_obj_set_size(driveRoller, 238, 220);							// Configure size & position of roller object
    lv_obj_align(driveRoller, LV_ALIGN_LEFT_MID, 0, 0);

    // Configure Partner Roller
    lv_roller_set_options(
        partnerRoller,
        "Dual Controllers\nSmart Partner (BETA)\nSimple Solo",
        LV_ROLLER_MODE_NORMAL
    );
    lv_roller_set_visible_row_count(partnerRoller, 2);
    lv_obj_set_style_bg_color(
        partnerRoller,
        lv_color_hex(0xFFFF00),
        LV_PART_SELECTED
    );
    lv_obj_set_style_text_color(
        partnerRoller,
        lv_color_hex(0x000000),
        LV_PART_SELECTED
    );
    lv_obj_set_size(partnerRoller, 238, 220);						// Configure size & position of roller object
    lv_obj_align(partnerRoller, LV_ALIGN_RIGHT_MID, 0, 0);
}

void UI::initUI(lv_obj_t* parent) {
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
    lv_obj_add_event_cb(colorRoller, UI::color_roller_event_handler, LV_EVENT_ALL, NULL);
};

void UI::color_roller_event_handler(lv_event_t* e) { // change color of color selector
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

void UI::goHome() { // go to home tab
    lv_tabview_set_act(tabview, 0, LV_ANIM_OFF);
}