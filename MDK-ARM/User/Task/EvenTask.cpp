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
        BSP::Remote::dr16.state_watch_.UpdateTime();
        BSP::Remote::dr16.state_watch_.CheckStatus();
        
        // 检查每个LK电机的状态
        for(int i = 0; i < 4; i++) {
            BSP::Motor::LK::Motor4005.getStateWatch(i + 1).UpdateTime();
            BSP::Motor::LK::Motor4005.getStateWatch(i + 1).CheckStatus();
        }
        
        // 检查每个DJI电机的状态
        for(int i = 0; i < 4; i++) {
            BSP::Motor::Dji::Motor3508.getStateWatch(i + 1).UpdateTime();
            BSP::Motor::Dji::Motor3508.getStateWatch(i + 1).CheckStatus();
        }
        Dir_Event.Notify();
        osDelay(5);
    }
}
bool Dir::Dir_Remote()
{
	// BSP::Remote::dr16.state_watch_.UpdateTime();
    // BSP::Remote::dr16.state_watch_.CheckStatus();
	
    Dir_Event.DirData.Dr16 = BSP::Remote::dr16.isDrOnline();
    return DirData.Dr16;
}

bool Dir::Dir_String()
{
    // 通过电机类提供的公共方法来判断是否在线（未断联）
    bool Dir = BSP::Motor::LK::Motor4005.isMotorOnline(0);
    for (int i = 0; i < 4; i++) {
        DirData.String[i] = BSP::Motor::LK::Motor4005.isMotorOnline(i);
    }

    return Dir;
}

bool Dir::Dir_Wheel()
{
    // 通过电机类提供的公共方法来判断是否在线（未断联）
    bool Dir = BSP::Motor::Dji::Motor3508.isMotorOnline(0);

    for (int i = 0; i < 4; i++) {
        DirData.Wheel[i] = BSP::Motor::Dji::Motor3508.isMotorOnline(i);
    }

    return Dir;
}

bool Dir::Dir_MeterPower()
{
    // bool Dir = MeterPower.isPmOnline();

    // DirData.MeterPower = Dir;

    // return Dir;
}

bool Dir::Dir_Communication()
{
    // DirData.Communication = Gimbal_to_Chassis_Data.isConnectOnline();
    // if (DirData.Communication == true) {
    //     Gimbal_to_Chassis_Data.Init();
    // }

    // return DirData.Communication;
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
    Dir_Remote();
    Dir_String();
    Dir_Wheel();
    Dir_MeterPower();
    Dir_Communication();
    Init_Flag();
    Dir_SuperCap();
}
