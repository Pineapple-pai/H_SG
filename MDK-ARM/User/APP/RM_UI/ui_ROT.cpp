//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.hpp"
#include "ui_ROT.hpp"
#include "../Task/CommunicationTask.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../BSP/SuperCap/SuperCap.hpp"

#define TOTAL_FIGURE 5
#define TOTAL_STRING 4

ui_interface_figure_t ui_ROT_now_figures[TOTAL_FIGURE];
uint8_t ui_ROT_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_ROT_now_strings[TOTAL_STRING];
uint8_t ui_ROT_dirty_string[TOTAL_STRING];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_ROT_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_ROT_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_ROT_now_figures, ui_ROT_dirty_figure, ui_ROT_now_strings, ui_ROT_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

// 保存上一次的底盘模式，用于比较是否需要更新UI
static int8_t last_chassis_mode = -1;

void ui_init_ROT() {
    ui_ROT_Chassis_left->figure_type = 0;
    ui_ROT_Chassis_left->operate_type = 1;
    ui_ROT_Chassis_left->layer = 0;
    ui_ROT_Chassis_left->color = 8;
    ui_ROT_Chassis_left->start_x = 536;
    ui_ROT_Chassis_left->start_y = 331;
    ui_ROT_Chassis_left->width = 1;
    ui_ROT_Chassis_left->end_x = 586;
    ui_ROT_Chassis_left->end_y = 751;

    ui_ROT_Chassis_right->figure_type = 0;
    ui_ROT_Chassis_right->operate_type = 1;
    ui_ROT_Chassis_right->layer = 0;
    ui_ROT_Chassis_right->color = 8;
    ui_ROT_Chassis_right->start_x = 1382;
    ui_ROT_Chassis_right->start_y = 331;
    ui_ROT_Chassis_right->width = 1;
    ui_ROT_Chassis_right->end_x = 1332;
    ui_ROT_Chassis_right->end_y = 751;

    ui_ROT_Gimbal_NewRect->figure_type = 1;
    ui_ROT_Gimbal_NewRect->operate_type = 1;
    ui_ROT_Gimbal_NewRect->layer = 0;
    ui_ROT_Gimbal_NewRect->color = 8;
    ui_ROT_Gimbal_NewRect->start_x = 661;
    ui_ROT_Gimbal_NewRect->start_y = 102;
    ui_ROT_Gimbal_NewRect->width = 1;
    ui_ROT_Gimbal_NewRect->end_x = 1261;
    ui_ROT_Gimbal_NewRect->end_y = 217;

    ui_ROT_Chassis_SC->figure_type = 4;
    ui_ROT_Chassis_SC->operate_type = 1;
    ui_ROT_Chassis_SC->layer = 0;
    ui_ROT_Chassis_SC->color = 3;  // 橙色
    ui_ROT_Chassis_SC->start_x = 854;
    ui_ROT_Chassis_SC->start_y = 539;
    ui_ROT_Chassis_SC->width = 1;
    ui_ROT_Chassis_SC->start_angle = 220;
    ui_ROT_Chassis_SC->end_angle = 220;  // 初始角度
    ui_ROT_Chassis_SC->rx = 286;
    ui_ROT_Chassis_SC->ry = 352;

    ui_ROT_Chassis_Power->figure_type = 4;
    ui_ROT_Chassis_Power->operate_type = 1;
    ui_ROT_Chassis_Power->layer = 0;
    ui_ROT_Chassis_Power->color = 0;  // 红色
    ui_ROT_Chassis_Power->start_x = 1064;
    ui_ROT_Chassis_Power->start_y = 539;
    ui_ROT_Chassis_Power->width = 1;
    ui_ROT_Chassis_Power->start_angle = 40;
    ui_ROT_Chassis_Power->end_angle = 40;  // 初始角度
    ui_ROT_Chassis_Power->rx = 286;
    ui_ROT_Chassis_Power->ry = 352;

    ui_ROT_Shoot_leave->figure_type = 7;
    ui_ROT_Shoot_leave->operate_type = 1;
    ui_ROT_Shoot_leave->layer = 1;
    ui_ROT_Shoot_leave->color = 8;
    ui_ROT_Shoot_leave->start_x = 1470;
    ui_ROT_Shoot_leave->start_y = 781;
    ui_ROT_Shoot_leave->width = 3;
    ui_ROT_Shoot_leave->font_size = 26;
    ui_ROT_Shoot_leave->str_length = 15;
    strcpy(ui_ROT_Shoot_leave->string, "当前剩余：");

    ui_ROT_Gimbal_Normal->figure_type = 7;
    ui_ROT_Gimbal_Normal->operate_type = 1;
    ui_ROT_Gimbal_Normal->layer = 1;
    ui_ROT_Gimbal_Normal->color = 2;  // 绿色
    ui_ROT_Gimbal_Normal->start_x = 730;
    ui_ROT_Gimbal_Normal->start_y = 164;
    ui_ROT_Gimbal_Normal->width = 2;
    ui_ROT_Gimbal_Normal->font_size = 22;
    ui_ROT_Gimbal_Normal->str_length = 3;
    strcpy(ui_ROT_Gimbal_Normal->string, "NOR");

    ui_ROT_Gimbal_Follow->figure_type = 7;
    ui_ROT_Gimbal_Follow->operate_type = 1;
    ui_ROT_Gimbal_Follow->layer = 1;
    ui_ROT_Gimbal_Follow->color = 2;  // 绿色
    ui_ROT_Gimbal_Follow->start_x = 1130;
    ui_ROT_Gimbal_Follow->start_y = 164;
    ui_ROT_Gimbal_Follow->width = 2;
    ui_ROT_Gimbal_Follow->font_size = 22;
    ui_ROT_Gimbal_Follow->str_length = 3;
    strcpy(ui_ROT_Gimbal_Follow->string, "FOL");

    ui_ROT_Gimbal_Rotating->figure_type = 7;
    ui_ROT_Gimbal_Rotating->operate_type = 1;
    ui_ROT_Gimbal_Rotating->layer = 1;
    ui_ROT_Gimbal_Rotating->color = 2;  // 绿色
    ui_ROT_Gimbal_Rotating->start_x = 930;
    ui_ROT_Gimbal_Rotating->start_y = 164;
    ui_ROT_Gimbal_Rotating->width = 2;
    ui_ROT_Gimbal_Rotating->font_size = 22;
    ui_ROT_Gimbal_Rotating->str_length = 3;
    strcpy(ui_ROT_Gimbal_Rotating->string, "ROT");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_ROT_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_ROT_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_ROT_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_ROT_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_ROT_last_figures[i] = ui_ROT_now_figures[i];
#endif
        ui_ROT_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_ROT_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_ROT_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_ROT_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_ROT_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_ROT_last_strings[i] = ui_ROT_now_strings[i];
#endif
        ui_ROT_dirty_string[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_ROT_now_figures[i].operate_type = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_ROT_now_strings[i].operate_type = 2;
    }
}

/**
 * @brief 更新底盘模式显示，高亮当前模式
 */
void ui_update_chassis_mode() {
    // 确定当前底盘模式
    int8_t current_mode = 0; // 默认模式
    
    if (Gimbal_to_Chassis_Data.getUniversal()) {
        current_mode = 1; // 万向模式 (Normal)
    } else if (Gimbal_to_Chassis_Data.getFollow()) {
        current_mode = 2; // 跟随模式 (Follow)
    } else if (Gimbal_to_Chassis_Data.getRotating()) {
        current_mode = 3; // 小陀螺模式 (Rotating)
    }
    
    // 只有在模式发生变化时才更新UI
    if (current_mode != last_chassis_mode) {
        // 先将所有模式文本设为绿色（默认状态）
        ui_ROT_Gimbal_Normal->color = 2;   // 绿色
        ui_ROT_Gimbal_Follow->color = 2;   // 绿色
        ui_ROT_Gimbal_Rotating->color = 2; // 绿色
        
        // 根据当前模式高亮对应文本为黄色
        switch(current_mode) {
            case 1: // Normal mode
                ui_ROT_Gimbal_Normal->color = 3; // 黄色高亮
                break;
            case 2: // Follow mode
                ui_ROT_Gimbal_Follow->color = 3; // 黄色高亮
                break;
            case 3: // Rotating mode
                ui_ROT_Gimbal_Rotating->color = 3; // 黄色高亮
                break;
            default:
                break;
        }
        
#ifdef MANUAL_DIRTY
        // 标记字符串为脏数据以便更新
        ui_ROT_Gimbal_Normal_dirty = 1;
        ui_ROT_Gimbal_Follow_dirty = 1;
        ui_ROT_Gimbal_Rotating_dirty = 1;
#endif
        
        // 更新上一次模式记录
        last_chassis_mode = current_mode;
    }
}

/**
 * @brief 更新功率显示（右侧红色弧线）
 */
void ui_update_power_display() {
    // 计算当前功率百分比并映射到角度
    // 假设功率限制为 120W，可以根据实际情况调整
    uint16_t power_percent = (BSP::Power::pm01.cin_power * 100) / 120;
    if (power_percent > 100) power_percent = 100;
    
    // 映射到角度 (40° 到 140°)
    uint16_t power_angle = 40 + (power_percent * 100) / 100;
    if (power_angle > 140) power_angle = 140;
    
    // 更新功率显示弧线
    if (ui_ROT_Chassis_Power->end_angle != power_angle) {
        ui_ROT_Chassis_Power->end_angle = power_angle;
        
#ifdef MANUAL_DIRTY
        ui_ROT_Chassis_Power_dirty = 1;
#endif
    }
}

/**
 * @brief 更新超级电容电量显示（左侧橙色弧线）
 */
void ui_update_super_cap() {
    // 获取超级电容电压并计算百分比
    // 假设电压范围为 12V-18V，可根据实际情况调整
    float voltage = BSP::SuperCap::cap.getCapVoltage();
    uint16_t voltage_percent = 0;
    
    if (voltage > 16.0f) {
        if (voltage > 18.0f) {
            voltage_percent = 100;
        } else {
            voltage_percent = (uint16_t)(((voltage - 16.0f) * 100) / 2.0f); // 映射到0-100%
        }
    }
    
    // 映射到角度 (220° 到 320°)
    uint16_t cap_angle = 220 + (voltage_percent * 100) / 100;
    if (cap_angle > 320) cap_angle = 320;
    
    // 更新超级电容显示弧线
    if (ui_ROT_Chassis_SC->end_angle != cap_angle) {
        ui_ROT_Chassis_SC->end_angle = cap_angle;
        
#ifdef MANUAL_DIRTY
        ui_ROT_Chassis_SC_dirty = 1;
#endif
    }
}

void ui_update_ROT() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_ROT_now_figures[i], &ui_ROT_last_figures[i], sizeof(ui_ROT_now_figures[i])) != 0) {
            ui_ROT_dirty_figure[i] = 1;
            ui_ROT_last_figures[i] = ui_ROT_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_ROT_now_strings[i], &ui_ROT_last_strings[i], sizeof(ui_ROT_now_strings[i])) != 0) {
            ui_ROT_dirty_string[i] = 1;
            ui_ROT_last_strings[i] = ui_ROT_now_strings[i];
        }
    }
#endif
    
    // 更新自定义UI元素
    ui_update_chassis_mode();
    ui_update_power_display();
    ui_update_super_cap();
    
    SCAN_AND_SEND();
}