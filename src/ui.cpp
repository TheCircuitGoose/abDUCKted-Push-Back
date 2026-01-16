#include "main.h"
#include "liblvgl/lvgl.h"
#include "pros/misc.hpp"

#include "project/ui.hpp"

UI::UI() {};

int UI::getAutonIndex() { // getters for selected values
    return lv_roller_get_selected(autonRoller); // return the index of roller
}
int UI::getColorIndex() {
    return lv_roller_get_selected(colorRoller);
}

void UI::initUI_NEW(lv_obj_t* parent) {
    tabview = lv_tabview_create(parent, LV_DIR_TOP, 40); // tabs
    tab_home = lv_tabview_add_tab(tabview, "Home");
    tab_auton = lv_tabview_add_tab(tabview, "Auton");
    tab_field = lv_tabview_add_tab(tabview, "Field");

    autonRoller = lv_roller_create(tab_auton); // rollers
    colorRoller = lv_roller_create(tab_auton);

    // HOME TAb
    lv_obj_set_size(tab_home, 480, 200);
    lv_obj_center(tab_home);

    // Team number label centered in left half
    team_label = lv_label_create(tab_home);
    lv_label_set_text(team_label, "9039T");
    lv_obj_set_style_text_font(team_label, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_set_style_text_align(team_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_width(team_label, 240);
    lv_obj_align(team_label, LV_ALIGN_LEFT_MID, 0, 0);

    // Battery percentage label on top (outside container)
    battery_percent_label = lv_label_create(tab_home);
    lv_label_set_text(battery_percent_label, "100%");
    lv_obj_set_style_text_font(battery_percent_label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_align(battery_percent_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_width(battery_percent_label, 200);
    lv_obj_align(battery_percent_label, LV_ALIGN_TOP_RIGHT, 0, 5);

    // Battery display container on right side
    battery_container = lv_obj_create(tab_home);
    lv_obj_set_size(battery_container, 120, 130);
    lv_obj_align(battery_container, LV_ALIGN_RIGHT_MID, -10, 15);
    lv_obj_set_style_bg_color(battery_container, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_border_color(battery_container, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_border_width(battery_container, 2, LV_PART_MAIN);
    lv_obj_set_style_pad_all(battery_container, 5, LV_PART_MAIN);
    lv_obj_set_style_radius(battery_container, 0, LV_PART_MAIN);

    // Green battery bar (fills the full rectangle)
    battery_bar = lv_bar_create(battery_container);
    lv_obj_set_size(battery_bar, 110, 120); // Fill the container
    lv_obj_center(battery_bar);
    lv_bar_set_range(battery_bar, 0, 100);
    lv_bar_set_value(battery_bar, 100, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(battery_bar, lv_color_hex(0x1a1a1a), LV_PART_MAIN);
    lv_obj_set_style_bg_color(battery_bar, lv_color_hex(0x00FF00), LV_PART_INDICATOR);
    lv_obj_set_style_anim_time(battery_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(battery_bar, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(battery_bar, 0, LV_PART_INDICATOR);

    // AUTON TAB
    lv_obj_set_size(tab_auton, 480, 200);
    lv_obj_center(tab_auton);
    lv_obj_set_style_pad_all(tab_auton, 0, LV_PART_MAIN);

    // Configure Auton Roller
    lv_roller_set_options(											// Configure Roller
        autonRoller, 
        auton::autonNames.c_str(), 
        LV_ROLLER_MODE_NORMAL
    );
    lv_roller_set_visible_row_count(autonRoller, 4);
    lv_obj_set_style_bg_color(											// Set highlight color of selected choice to a bold yellow
        autonRoller, 
        lv_color_hex(0xFFFF00), 
        LV_PART_SELECTED
    );
    lv_obj_set_style_text_color(										// Set text color to black
        autonRoller, 
        lv_color_hex(0x000000), 
        LV_PART_SELECTED
    );
    lv_obj_set_width(autonRoller, lv_pct(50));
    lv_obj_set_height(autonRoller, lv_pct(100));
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
    lv_obj_set_width(colorRoller, lv_pct(50));
    lv_obj_set_height(colorRoller, lv_pct(100));
    lv_obj_align(colorRoller, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(colorRoller, color_roller_event_handler, LV_EVENT_ALL, NULL);

    // Create timer to update battery display every 10 seconds
    lv_timer_create(UI::battery_task, 10000, (void*)this);
}

void UI::color_roller_event_handler(lv_event_t* e) { // change color of color selector
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * roller = static_cast<lv_obj_t *>(lv_event_get_target(e));

    if (code == LV_EVENT_VALUE_CHANGED) {
        int selected = lv_roller_get_selected(roller);
        if (selected == 1) { // green for blanking
            lv_obj_set_style_bg_color(roller, lv_color_hex(0x00FF00), LV_PART_SELECTED);
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

void UI::goAuton() { // go to auton tab
    lv_tabview_set_act(tabview, 1, LV_ANIM_OFF);
}

void UI::goField() { // go to field tab
    lv_tabview_set_act(tabview, 2, LV_ANIM_OFF);
}

void UI::battery_task(lv_timer_t* timer) { // update battery display
    UI* ui = (UI*)timer->user_data;
    
    // Use PROS battery level (already in percent 0-100)
    int battery_percent = pros::battery::get_capacity();
    
    if (battery_percent < 0) battery_percent = 0;
    if (battery_percent > 100) battery_percent = 100;
    
    lv_bar_set_value(ui->battery_bar, battery_percent, LV_ANIM_OFF);
    
    static char buf[32];
    snprintf(buf, sizeof(buf), "%d%%", battery_percent);
    lv_label_set_text(ui->battery_percent_label, buf);
}