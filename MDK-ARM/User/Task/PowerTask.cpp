#include "PowerTask.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../BSP/SuperCap/SuperCap.hpp"
#include "../Task/CommunicationTask.hpp"
#include "../APP/Tools.hpp"
#include "../APP/Variable.hpp"
#include "../BSP/Motor/Lk/Lk_motor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "cmsis_os2.h"
#include "math.h"
#include "../BSP/stdxxx.hpp"

using BSP::Motor::Dji::GM3508;
using BSP::Motor::LK::LK4005;
using namespace SGPowerControl;
SGPowerControl::PowerTask_t PowerControl;

uint32_t pm01_ms = 0;
float W2, T2;
float EffectivePower_t;
uint16_t time1 = 2;
uint8_t power  = 100;

void RLSTask(void *argument)
{
    osDelay(500);
    
    // 初始化电机接口
    PowerControl.InitMotorInterfaces(BSP::Motor::Dji::Motor3508, BSP::Motor::LK::Motor4005);
    
    float prev_e_t = 0.0f;
    float dt = 0.001f;  // 1ms周期
    
    for (;;) {
        // 轮向电机功率计算 (DJI 3508)
        PowerControl.Wheel_PowerData.UpRLS(pid_vel_Wheel, toque_const_3508, rpm_to_rads_3508);
        
        // 舵向电机功率计算 (LK 4005)
        PowerControl.String_PowerData.UpRLS(pid_vel_String, toque_const_4005, rpm_to_rads_4005);

        // 获取裁判系统反馈的缓冲能量（单位：J）
        float buffer_energy = ext_power_heat_data_0x0202.chassis_power_buffer;  // 转换为焦耳

        // 更新能量环，使用裁判反馈的缓冲能量
        //PowerControl.Wheel_PowerData.UpdateEnergy(buffer_energy, dt);
        float lim_cin_power = 60 - 1.0f;
        // 获取当前最大功率限制
        //float max_power_limit = PowerControl.Wheel_PowerData.GetMaxPowerLimit();

        // 发送CAN指令
        BSP::SuperCap::cap.SetSendValue(lim_cin_power);
        BSP::SuperCap::cap.sendCAN(&hcan2, CAN_TX_MAILBOX0);

        osDelay(5);
    }
}

// 统一的功率计算方法
void PowerUpData_t::UpRLS(PID *pid, const float toque_const, const float rpm_to_rads)
{
    if (!motor_interface_) return;
    
    EffectivePower = 0.0f;
    
    if(Init_flag == true)
    {
        samples[0][0] = 0.0f;
        samples[1][0] = 0.0f;
    }

    for (int i = 0; i < 4; i++) {
        EffectivePower +=
            motor_interface_->GetCurrent(i) * motor_interface_->GetSpeed(i) * toque_const * rpm_to_rads;

        samples[0][0] += fabs(motor_interface_->GetSpeed(i)) * rpm_to_rads;
        samples[1][0] +=
            motor_interface_->GetCurrent(i) * motor_interface_->GetCurrent(i) * toque_const * toque_const;
    }

    if (is_RLS == true && Dir_Event.getSuperCap() == false && Dir_Event.GetDir_String() == false) {
        //        params = rls.update(samples, BSP::SuperCap::cap.getOutPower() - EffectivePower - k3);
        params = rls.update(samples, BSP::Power::pm01.cin_power - EffectivePower - k3);

        // }
        k1 = fmax(params[0][0], 1e-5f); 
        k2 = fmax(params[1][0], 1e-5f); 
    }

    Cur_EstimatedPower = k1 * samples[0][0] + k2 * samples[1][0] + EffectivePower + k3;

    EstimatedPower = 0;
    for (int i = 0; i < 4; i++) {
        Initial_Est_power[i] = pid->GetCout() * toque_const * motor_interface_->GetSpeed(i) * rpm_to_rads +
                               +fabs(motor_interface_->GetSpeed(i) * rpm_to_rads) * k1 +
                               pid->GetCout() * toque_const * pid->GetCout() * toque_const * k2 + k3 / 4.0f;

        if (Initial_Est_power[i] < 0) 
            continue;

        EstimatedPower += Initial_Est_power[i];
    }
    
    Init_flag = true;
}

// 等比缩放的最大分配功率
void PowerUpData_t::UpScaleMaxPow(PID *pid)
{
    float sumErr = 0.0f;
    int motor_count = 4; 

    for (int i = 0; i < motor_count; i++) {
        sumErr += fabsf(pid[i].GetErr());
    }

    if (sumErr < 1e-5f) {
        for (int i = 0; i < motor_count; i++) {
            pMaxPower[i] = MAXPower / motor_count;
        }
        return;
    }

    for (int i = 0; i < motor_count; i++) {
        pMaxPower[i] = MAXPower * (fabsf(pid[i].GetErr()) / sumErr);
    }
}

// 计算应分配的力矩
void PowerUpData_t::UpCalcMaxTorque(float *final_Out, PID *pid, const float toque_const, const float rpm_to_rads)
{
    if (!motor_interface_) return;
    
    if (EstimatedPower > MAXPower) 
    {
        for (int i = 0; i < 4; i++) {
            float omega = motor_interface_->GetSpeed    (i) * rpm_to_rads;

            float A = k2;
            float B = omega;
            float C = k1 * fabsf(omega) + k3 / 4.0f - pMaxPower[i];

            float delta = (B * B) - 4.0f * A * C;

            if (delta <= 0) {
                Cmd_MaxT[i] = -B / (2.0f * A) / toque_const;
            } else {
                Cmd_MaxT[i] = pid[i].GetCout() > 0.0f ? (-B + sqrtf(delta)) / (2.0f * A) / toque_const
                                                      : (-B - sqrtf(delta)) / (2.0f * A) / toque_const;
            }

            Cmd_MaxT[i] = Tools.clamp(static_cast<float>(Cmd_MaxT[i]), 
                         motor_interface_->GetMaxCurrent(), 
                         -motor_interface_->GetMaxCurrent());

            final_Out[i] = static_cast<float>(Cmd_MaxT[i]);
        }
    }
}

// 新增：能量环相关方法实现
// void PowerUpData_t::UpdateEnergy(float power, float dt)
// {
//     // 假设电容最大输出为300W，因此用户输入的最大底盘功率输出为裁判系统功率上限+300W
//     float energy_change = power * dt;  // 单位：焦耳
//     Energy += energy_change;

//     // 限制能量在0到60J之间
//     Energy = fmax(0.0f, fmin(Energy, 60.0f));
// }

// float PowerUpData_t::GetAvailableEnergy() const
// {
//     return Energy;
// }

// float PowerUpData_t::GetMaxPowerLimit() const
// {
//     // 根据能量状态动态调整最大功率限制
//     float e_t = sqrtf(E_upper) - sqrtf(Energy);  // e(t) = √E_s - √E_f
//     float p_max = refMaxPower - Kp * e_t - Kd * (e_t - e_t_prev) / dt;

//     // 防止功率下限过低
//     p_max = fmax(p_max, MIN_MAXPOWER_CONFIGURED);

//     return p_max;
// }
