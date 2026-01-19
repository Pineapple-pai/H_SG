#include "EvenTask.hpp"
#include "../APP/Buzzer.hpp"
#include "../APP/LED.hpp"
#include "../BSP/Remote/Dbus.hpp"
#include "../BSP/Init.hpp"
#include "../Task/CommunicationTask.hpp"
#include "../BSP/SuperCap/SuperCap.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../APP/Variable.hpp"
#include "cmsis_os2.h"
#include "tim.h"
#include "../BSP/state_watch.hpp"
#include "../BSP/Motor/Lk/Lk_motor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Common/StateWatch/buzzer_manager.hpp"
// using namespace Event;

Dir Dir_Event;

auto LED_Event    = std::make_unique<LED>(&Dir_Event); // 让LED灯先订阅，先亮灯再更新蜂鸣器
auto Buzzer_Event = std::make_unique<Buzzer>(&Dir_Event);

void DirUpdata()
{
    Dir_Event.UpEvent();
}

void EventTask(void *argument)
{
    osDelay(500);
    for (;;) {

        Dir_Event.Notify();
        osDelay(5);
    }
}
// bool Dir::Dir_Remote()
// {
// 	// BSP::Remote::dr16.state_watch_.UpdateTime();
//     // BSP::Remote::dr16.state_watch_.CheckStatus();
	
//     Dir_Event.DirData.Dr16 = BSP::Remote::dr16.isDrOnline();
//     return DirData.Dr16;
// }

/**
 * @brief 检测舵向电机断联状态
 * 
 * @return true 所有舵向电机在线
 * @return false 有舵向电机离线
 */
bool Dir::Dir_String()
{
    bool allOnline = true;
    for (int i = 0; i < 4; i++) {
        DirData.String[i] = BSP::Motor::LK::Motor4005.isMotorOnline(i + 1);
        if (!DirData.String[i]) {
            allOnline = false;
            // 请求蜂鸣器报警，鸣叫电机ID次数
            BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestMotorRing(i + 1);
        }
    }
    return allOnline;
}

/**
 * @brief 检测轮向电机断联状态
 * 
 * @return true 所有轮向电机在线
 * @return false 有轮向电机离线
 */
bool Dir::Dir_Wheel()
{
    bool allOnline = true;
    for (int i = 0; i < 4; i++) {
        DirData.Wheel[i] = BSP::Motor::Dji::Motor3508.isMotorOnline(i + 1);
        if (!DirData.Wheel[i]) {
            allOnline = false;
            // 请求蜂鸣器报警，鸣叫电机ID次数
            BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestMotorRing(i + 1);
        }
    }
    return allOnline;
}

bool Dir::Dir_MeterPower()
{
    // bool Dir = MeterPower.isPmOnline();

    // DirData.MeterPower = Dir;

    // return Dir;
}

/**
 * @brief 检测板间通信断联状态
 * 
 * @return true 板间通信在线
 * @return false 板间通信离线
 */
bool Dir::Dir_Communication()
{
    DirData.Communication = Gimbal_to_Chassis_Data.isConnectOnline();
    if (!DirData.Communication) {
        // 请求蜂鸣器报警，板间通信鸣叫3次长音
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestCommunicationRing();
    }
    return DirData.Communication;
}

bool Dir::Dir_SuperCap()
{
    // bool Dir = BSP::SuperCap::cap.isScOnline() && BSP::Power::pm01.isPmOnline();

    // DirData.SuperCap = Dir;

    // return Dir;
}

bool Dir::Init_Flag()
{
    DirData.InitFlag = InitFlag;
    return InitFlag;
}

/**
 * @brief 更新事件
 *
 */
void Dir::UpEvent()
{
    //Dir_Remote();
    Dir_String();
    Dir_Wheel();
    //Dir_MeterPower();
    Dir_Communication();
    Init_Flag();
    //Dir_SuperCap();
    
    // 更新蜂鸣器管理器
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
}
