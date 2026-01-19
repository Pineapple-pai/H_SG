#include "CallBack.hpp"
#include "../APP/Referee/RM_RefereeSystem.h"
#include "../BSP/Remote/Dbus.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../BSP/SuperCap/SuperCap.hpp"
#include "../Task/CommunicationTask.hpp"
#include "Variable.hpp"
#include "../HAL/CAN/can_hal.hpp"
#include "../HAL/UART/uart_hal.hpp"
#include <algorithm>

uint8_t dbus_rx_buffer[18];
uint8_t referee_rx_buffer[18];

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame1;
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    if (hcan == can1.get_handle())
    {
        can1.receive(rx_frame1);  // receive()内部会自动触发所有注册的回调
        BSP::Motor::LK::Motor4005.Parse(rx_frame1);
        BSP::Motor::Dji::Motor3508.Parse(rx_frame1);
        
    }
}
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame2;
    auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    if (hcan == can2.get_handle())
    {
        can2.receive(rx_frame2);  // receive()内部会自动触发所有注册的回调
        
		//BSP::Motor::LK::Motor4005.Parse(rx_frame2);	
        Gimbal_to_Chassis_Data.HandleCANMessage(rx_frame2.id, rx_frame2.data);
        BSP::Power::PM01ParseDate(rx_frame2);
    }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 获取UART实例
    auto& uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
    auto& uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    if(huart == uart3.get_handle()) {
        // 调用您的解析函数
	      BSP::Remote::dr16.Parse(huart, Size);
        HAL::UART::Data dbus_rx_data{dbus_rx_buffer, sizeof(dbus_rx_buffer)};
        //uart3.receive_dma_idle(dbus_rx_data);
    }
    else if(huart == uart6.get_handle())
    {
        RM_RefereeSystem::RM_RefereeSystemParse(huart);
        HAL::UART::Data referee{referee_rx_buffer, sizeof(referee_rx_buffer)};
    }
}
