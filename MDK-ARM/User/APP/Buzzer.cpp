
#include "../App/Buzzer.hpp"

#include "cmsis_os2.h"
#include "tim.h"

/**
 * @brief          控制蜂鸣器定时器的分频和重载值
 * @param[in]      psc，设置定时器的分频系数
 * @param[in]      pwm，设置定时器的重载值
 * @retval         none
 */
void Buzzer::buzzer_on(uint16_t psc, uint16_t pwm)
{
//  __HAL_TIM_PRESCALER(&htim4, psc);
//  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
}

/**
 * @brief          关闭蜂鸣器
 * @param[in]      none
 * @retval         none
 */
void Buzzer::buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

bool Buzzer::Update()
{
    Dir *dir = static_cast<Dir *>(sub);

    dir_t[0] = dir->GetDir_Remote();

    if (dir->Ger_Init_Flag() && buzzerInit == false)
    {
        SYSTEM_START();

        buzzerInit = true;
    }

    // 在板间通信模式下，使用板间通信状态判断是否正常
    // 因为遥控器连接在云台板，底盘板通过板间通信获取控制信号
    bool remote_online = dir->getDir_Communication();  // 改用板间通信状态

    static bool last_remote_online = true;

    if (remote_online && !last_remote_online) 
    {      
        STOP();  // 遥控器复连接，关闭蜂鸣器
    }

    last_remote_online = remote_online;
    
    if (remote_online) 
    {
        // 检查是否有电机断联，如果有则持续报警
        static uint32_t last_beep_time = 0;
        uint32_t current_time = osKernelGetTickCount();
        
        // 查找第一个断联的电机并获取其ID
        int disconnected_motor_id = 0;  // 改为0表示没有断联
        
        for (int i = 0; i < 4; i++) {
            if (!dir->DirData.String[i]) {
                disconnected_motor_id = i + 1; // 电机ID为1-4
                break;
            }
        }
        if (disconnected_motor_id == 0) {
            for (int i = 0; i < 4; i++) {
                if (!dir->DirData.Wheel[i]) {
                    disconnected_motor_id = i + 1; // 电机ID为1-4
                    break;
                }
            }
        }        
        // 如果有电机断联，每隔一段时间鸣叫对应次数
        if (disconnected_motor_id > 0 && !is_busy && (current_time - last_beep_time > 2000)) {
            B(disconnected_motor_id); // 按电机ID鸣叫对应次数
            last_beep_time = current_time;
        }
        
        osDelay(10);
        return true;
    } else {
        if (!is_busy) {
            B___(); // 长鸣
        }
        osDelay(10);
        return false;
    }

    osDelay(10);
    return true;
}

void Buzzer::STOP()
{
    buzzer_off();
    is_busy = false;
}

void Buzzer::SYSTEM_START()
{
    STOP();

    is_busy = true;

    buzzer_on(1, 10000);
    osDelay(350);
    buzzer_off();
    osDelay(100);
    buzzer_on(6, 10000);
    osDelay(250);
    buzzer_on(1, 10000);
    osDelay(500);
    buzzer_off();
    is_busy = false;
}

void Buzzer::B(uint8_t num)
{
    switch (num)
    {
    case 1:
        B_();
        break;
    case 2:
        B_B_();
        break;
    case 3:
        B_B_B_();
        break;
    case 4:
        B_B_B_B_();
    default:
        buzzer_off();
        break;
    }
}

void Buzzer::B_()
{
    STOP();
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(150);  // 从50ms增加到150ms
    buzzer_off();
    osDelay(850);  // 从950ms减少到850ms，保持总周期1秒
    is_busy = false;
}

void Buzzer::B_B_()
{
    STOP();
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(150);  // 增加鸣叫时间
    buzzer_off();
    osDelay(200);  // 增加间隔时间
    buzzer_on(1, 10000);
    osDelay(150);  // 增加鸣叫时间
    buzzer_off();
    osDelay(500);  // 调整总间隔时间
    is_busy = false;
}

void Buzzer::B_B_B_()
{
    STOP();
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(150);  // 增加鸣叫时间
    buzzer_off();
    osDelay(150);  // 增加间隔时间

    buzzer_on(1, 10000);
    osDelay(150);  // 增加鸣叫时间
    buzzer_off();
    osDelay(150);  // 增加间隔时间

    buzzer_on(1, 10000);
    osDelay(150);  // 增加鸣叫时间
    buzzer_off();
    osDelay(400);  // 调整总间隔时间

    is_busy = false;
}

void Buzzer::B_B_B_B_()
{
    STOP();
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(150);  // 增加鸣叫时间
    buzzer_off();
    osDelay(150);  // 增加间隔时间

    buzzer_on(1, 10000);
    osDelay(150);  // 增加鸣叫时间
    buzzer_off();
    osDelay(150);  // 增加间隔时间

    buzzer_on(1, 10000);
    osDelay(150);  // 增加鸣叫时间
    buzzer_off();
    osDelay(150);  // 增加间隔时间

    buzzer_on(1, 10000);
    osDelay(150);  // 增加鸣叫时间
    buzzer_off();
    osDelay(250);  // 调整总间隔时间

    is_busy = false;
}

void Buzzer::B___()
{
    STOP();
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(800);  // 增加长鸣时间
    is_busy = false;
}

void Buzzer::B_CONTINUE()
{
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(150);  // 增加鸣叫时间
    buzzer_off();
    osDelay(100);  // 调整间隔时间

    is_busy = false;
}