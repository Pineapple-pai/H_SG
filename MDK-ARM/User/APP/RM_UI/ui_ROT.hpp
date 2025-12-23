//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_ROT_H
#define UI_ROT_H

#include "ui_interface.hpp"

extern ui_interface_figure_t ui_ROT_now_figures[5];
extern uint8_t ui_ROT_dirty_figure[5];
extern ui_interface_string_t ui_ROT_now_strings[4];
extern uint8_t ui_ROT_dirty_string[4];

#define ui_ROT_Chassis_left ((ui_interface_line_t*)&(ui_ROT_now_figures[0]))
#define ui_ROT_Chassis_right ((ui_interface_line_t*)&(ui_ROT_now_figures[1]))
#define ui_ROT_Gimbal_NewRect ((ui_interface_rect_t*)&(ui_ROT_now_figures[2]))
#define ui_ROT_Chassis_SC ((ui_interface_arc_t*)&(ui_ROT_now_figures[3]))
#define ui_ROT_Chassis_Power ((ui_interface_arc_t*)&(ui_ROT_now_figures[4]))

#define ui_ROT_Shoot_leave (&(ui_ROT_now_strings[0]))
#define ui_ROT_Gimbal_Normal (&(ui_ROT_now_strings[1]))
#define ui_ROT_Gimbal_Follow (&(ui_ROT_now_strings[2]))
#define ui_ROT_Gimbal_Rotating (&(ui_ROT_now_strings[3]))

#ifdef MANUAL_DIRTY
#define ui_ROT_Chassis_left_dirty (ui_ROT_dirty_figure[0])
#define ui_ROT_Chassis_right_dirty (ui_ROT_dirty_figure[1])
#define ui_ROT_Gimbal_NewRect_dirty (ui_ROT_dirty_figure[2])
#define ui_ROT_Chassis_SC_dirty (ui_ROT_dirty_figure[3])
#define ui_ROT_Chassis_Power_dirty (ui_ROT_dirty_figure[4])

#define ui_ROT_Shoot_leave_dirty (ui_ROT_dirty_string[0])
#define ui_ROT_Gimbal_Normal_dirty (ui_ROT_dirty_string[1])
#define ui_ROT_Gimbal_Follow_dirty (ui_ROT_dirty_string[2])
#define ui_ROT_Gimbal_Rotating_dirty (ui_ROT_dirty_string[3])
#endif

void ui_init_ROT();
void ui_update_ROT();

#endif // UI_ROT_H