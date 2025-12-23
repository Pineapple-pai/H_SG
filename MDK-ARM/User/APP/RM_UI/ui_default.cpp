//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.hpp"
#include "ui_default.hpp"

#define TOTAL_FIGURE 0
#define TOTAL_STRING 0

#define SCAN_AND_SEND() ui_scan_and_send(ui_default_now_figures, ui_default_dirty_figure, ui_default_now_strings, ui_default_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_default_now_figures[TOTAL_FIGURE];
uint8_t ui_default_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_default_now_strings[TOTAL_STRING];
uint8_t ui_default_dirty_string[TOTAL_STRING];
#endif


void ui_init_default() {
    uint32_t idx = 0;

    SCAN_AND_SEND();

}

void ui_update_default() {
#ifndef MANUAL_DIRTY
#endif
    SCAN_AND_SEND();
}