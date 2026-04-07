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

auto LED_Event    = std::make_unique<LED>(&Dir_Event);     // 先更新 LED，再更新蜂鸣器
auto Buzzer_Event = std::make_unique<Buzzer>(&Dir_Event);

void DirUpdata()
{
    Dir_Event.UpEvent();
}

void EventTask(void *argument)
{
    osDelay(500);
    for (;;)
    {
        Dir_Event.Notify();
        osDelay(5);
    }
}

// bool Dir::Dir_Remote()
// {
//     // BSP::Remote::dr16.state_watch_.UpdateTime();
//     // BSP::Remote::dr16.state_watch_.CheckStatus();
//     Dir_Event.DirData.Dr16 = BSP::Remote::dr16.isDrOnline();
//     return DirData.Dr16;
// }

/**
 * @brief 检测舵向电机是否掉线
 * @return true 所有舵向电机在线
 * @return false 存在舵向电机掉线
 */
bool Dir::Dir_String()
{
    // 直接参考:
    // 1. 在线检测接口:
    //    BSP::Motor::LK::Motor4005.isMotorOnline(i + 1)。
    // 2. 蜂鸣器提示接口:
    //    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestMotorRing(i + 1)。
    // TODO(教学挖空): 在这里补全舵向电机在线检测。
    // 提示:
    // 1. 逐个检查 4 个舵向电机是否在线。
    // 2. 把结果写入 DirData.String[i]。
    // 3. 如有掉线，可选择触发蜂鸣器提示。
    // 提示词:
    // "请补全舵向电机在线检测，并在掉线时给出提示。"
    for (int i = 0; i < 4; ++i) {
        DirData.String[i] = true;
    }
    return true;
    bool allOnline = true;
    for (int i = 0; i < 4; i++) {
        DirData.String[i] = BSP::Motor::LK::Motor4005.isMotorOnline(i + 1);
        if (!DirData.String[i]) {
            allOnline = false;
            // 请求蜂鸣器按电机编号进行提示
            BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestMotorRing(i + 1);
        }
    }
    return allOnline;
}

/**
 * @brief 检测轮向电机是否掉线
 * @return true 所有轮向电机在线
 * @return false 存在轮向电机掉线
 */
bool Dir::Dir_Wheel()
{
    // 直接参考:
    // 1. 在线检测接口:
    //    BSP::Motor::Dji::Motor3508.isMotorOnline(i + 1)。
    // 2. 蜂鸣器提示接口同样可以用 requestMotorRing(i + 1)。
    // TODO(教学挖空): 在这里补全驱动轮电机在线检测。
    // 提示词:
    // "请补全驱动轮电机在线检测，并把检测结果写入 DirData.Wheel。"
    for (int i = 0; i < 4; ++i) {
        DirData.Wheel[i] = true;
    }
    return true;
    bool allOnline = true;
    for (int i = 0; i < 4; i++) {
        DirData.Wheel[i] = BSP::Motor::Dji::Motor3508.isMotorOnline(i + 1);
        if (!DirData.Wheel[i]) {
            allOnline = false;
            // 请求蜂鸣器按电机编号进行提示
            BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestMotorRing(i + 1);
        }
    }
    return allOnline;
}

bool Dir::Dir_MeterPower()
{
    // 直接参考:
    // 1. 功率计在线检测接口:
    //    BSP::Power::pm01.isPmOnline()。
    // TODO(教学挖空): 在这里补全功率计在线检测。
    // 提示词:
    // "请补全功率计在线检测，并同步更新 DirData.MeterPower。"
    DirData.MeterPower = true;
    return true;
    bool Dir = BSP::Power::pm01.isPmOnline();
    DirData.MeterPower = Dir;
    return Dir;
}

/**
 * @brief 检测板间通信是否掉线
 * @return true 板间通信在线
 * @return false 板间通信离线
 */
bool Dir::Dir_Communication()
{
    // 直接参考:
    // 1. 板间通信在线检测接口:
    //    Gimbal_to_Chassis_Data.isConnectOnline()。
    // 2. 掉线提示接口:
    //    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestCommunicationRing()。
    // TODO(教学挖空): 在这里补全板间通信在线检测。
    // 提示词:
    // "请补全板间通信在线检测，必要时在掉线后触发提示。"
    DirData.Communication = true;
    return true;
    DirData.Communication = Gimbal_to_Chassis_Data.isConnectOnline();
    if (!DirData.Communication) {
        // 请求蜂鸣器进行板间通信掉线提示
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestCommunicationRing();
    }
    return DirData.Communication;
}

bool Dir::Dir_SuperCap()
{
    // 直接参考:
    // 1. 超级电容在线检测:
    //    BSP::SuperCap::cap.isScOnline()。
    // 2. 如果你希望“超级电容 + 功率计”一起判在线，可继续组合
    //    BSP::Power::pm01.isPmOnline()。
    // TODO(教学挖空): 在这里补全超级电容在线检测。
    // 提示词:
    // "请补全超级电容在线检测，并更新 DirData.SuperCap。"
    DirData.SuperCap = true;
    return true;
    bool Dir = BSP::SuperCap::cap.isScOnline() && BSP::Power::pm01.isPmOnline();
    DirData.SuperCap = Dir;
    return Dir;
}

bool Dir::Init_Flag()
{
    DirData.InitFlag = InitFlag;
    return InitFlag;
}

/**
 * @brief 更新事件状态
 */
void Dir::UpEvent()
{
    // Dir_Remote();
    Dir_String();
    Dir_Wheel();
    // Dir_MeterPower();
    Dir_Communication();
    Init_Flag();
    Dir_SuperCap();

    // 更新蜂鸣器请求队列
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
}
