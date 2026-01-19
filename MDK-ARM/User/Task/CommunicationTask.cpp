#include "CommunicationTask.hpp"
#include "../APP/Referee/RM_RefereeSystem.h"
#include "cmsis_os2.h"
// #include "Variable.hpp"
// #include "State.hpp"
#include "tim.h"
#include "../BSP/Remote/Dbus.hpp"
#include "usart.h"
#include "../HAL/UART/uart_hal.hpp"
#include "../BSP/state_watch.hpp"
#include "../HAL/CAN/can_hal.hpp"
#define SIZE 8
extern uint8_t dbus_rx_buffer[18];
int a=0;

// 添加状态监视器，500ms超时
BSP::WATCH_STATE::StateWatch state_watch_(500);
void CommunicationTask(void *argument)
{
	osDelay(500);
    for (;;)
    {
        Gimbal_to_Chassis_Data.Transmit();
        osDelay(10);
    }
}
Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

namespace Communicat
{
void Gimbal_to_Chassis::Init()
{
    frame1_received = false;
    frame2_received = false;
    last_frame_time = HAL_GetTick();
    // 初始化状态监视器的时间戳，避免启动时误判为离线
    state_watch_.UpdateLastTime();
    state_watch_.UpdateTime();
}
void Gimbal_to_Chassis::HandleCANMessage(uint32_t std_id, uint8_t* data)
{
    ParseCANFrame(std_id, data);
}

void Gimbal_to_Chassis::ParseCANFrame(uint32_t std_id, uint8_t* data)
{
    uint32_t current_time = HAL_GetTick();
    
    // 检查超时，如果超时则重置接收状态
    // if (current_time - last_frame_time > FRAME_TIMEOUT) {
    //     frame1_received = false;
    //     frame2_received = false;
    // }
    // this->last_frame_time = current_time;

    switch(std_id) {
        case CAN_G2C_FRAME1_ID:
            std::memcpy(can_rx_buffer, data, 8);
            frame1_received = true;
            break;
        case CAN_G2C_FRAME2_ID:
            std::memcpy(can_rx_buffer + 8, data, 8);
            frame2_received = true;
            break;
        default:
            return;
    }

    // 如果三帧都接收完成，处理数据
    if (frame1_received || frame2_received) {
        ProcessReceivedData();

        state_watch_.UpdateLastTime();
        state_watch_.UpdateTime();
        state_watch_.CheckStatus();
        // 重置接收状态
        frame1_received = false;
        frame2_received = false;
    }
}
void Gimbal_to_Chassis::ProcessReceivedData()
{
    const uint8_t EXPECTED_HEAD = 0xA5; // 根据发送端设置的头字节
    const uint8_t EXPECTED_LEN = 1 + sizeof(Direction) + sizeof(ChassisMode) + sizeof(UiList);

    if (can_rx_buffer[0] != EXPECTED_HEAD) {
        return;
    }
    auto ptr = can_rx_buffer + 1; // 跳过头字节

    std::memcpy(&direction, ptr, sizeof(direction));
    ptr += sizeof(direction);

    std::memcpy(&chassis_mode, ptr, sizeof(chassis_mode));
    ptr += sizeof(chassis_mode);

    std::memcpy(&ui_list, ptr, sizeof(ui_list));
    ptr += sizeof(ui_list);
}

void Gimbal_to_Chassis::SlidingWindowRecovery()
{

    const int window_size = sizeof(pData);
    int found_pos = -1;

    // 遍历整个缓冲区寻找有效头
    for (int i = 0; i < window_size; i++) // 修正循环条件
    {
        if (pData[i] == 0xA5)
        {
            found_pos = i;
            break;
        }
    }

    if (found_pos > 0)
    {
        // 使用memmove处理可能重叠的内存区域
        std::memmove(pData, &pData[found_pos], window_size - found_pos);

        // 可选：重新配置DMA接收剩余空间
        int remaining_space = window_size - found_pos;
        HAL_UART_Receive_DMA(&huart1, pData + remaining_space, found_pos);
    }
}

bool Gimbal_to_Chassis::isConnectOnline()
{
    // 先更新当前时间，再检查状态，确保判断基于最新时间
    state_watch_.UpdateTime();
    state_watch_.CheckStatus();
    return (state_watch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE);
}

void Gimbal_to_Chassis::Transmit()
{
    // 使用临时指针将数据拷贝到缓冲区
    booster.heat_one = 0x21;
    booster.heat_two = 0x12;

    setNowBoosterHeat(ext_power_heat_data_0x0202.shooter_id1_17mm_cooling_heat);
    setBoosterMAX(ext_power_heat_data_0x0201.shooter_barrel_heat_limit);
    setBoosterCd(ext_power_heat_data_0x0201.shooter_barrel_cooling_value);

    // 使用临时指针将数据拷贝到缓冲区
    uint8_t tx_data[8];
    auto temp_ptr = tx_data;

    const auto memcpy_safe = [&](const auto &data) {
        std::memcpy(temp_ptr, &data, sizeof(data));
        temp_ptr += sizeof(data);
    };
    memcpy_safe(booster.heat_one);       
    memcpy_safe(booster.heat_two);       
    memcpy_safe(booster.booster_heat_cd);  
    memcpy_safe(booster.booster_heat_max);  
    memcpy_safe(booster.booster_now_heat);  
    HAL::CAN::Frame frame;
    frame.dlc = 8;
    frame.is_extended_id = false;
    frame.is_remote_frame = false;
    frame.id = CAN_CHASSIS_TO_GIMBAL_ID;  // 0x501
    std::memcpy(frame.data, tx_data, 8);

    auto& can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    can2.send(frame);

}

}; // namespace Communicat