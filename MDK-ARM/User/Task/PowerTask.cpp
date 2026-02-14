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
#include "../APP/Variable.hpp"
using BSP::Motor::Dji::GM3508;
using BSP::Motor::LK::LK4005;
using namespace SGPowerControl;
SGPowerControl::PowerTask_t PowerControl;

uint32_t pm01_ms = 0;
float W2, T2;
float EffectivePower_t;
uint16_t time1 = 2;
uint8_t power  = 100;
float test_max = 60.0f;

void RLSTask(void *argument)
{
    osDelay(500);
    
    // 初始化电机接口
    PowerControl.InitMotorInterfaces(BSP::Motor::Dji::Motor3508, BSP::Motor::LK::Motor4005);
    
    float prev_e_t = 0.0f;
    float dt = 0.001f;  // 1ms周期

    for (;;) {
//        if (ext_power_heat_data_0x0201.chassis_power_limit > 0) {
//            PowerControl.Wheel_PowerData.MAXPower = ext_power_heat_data_0x0201.chassis_power_limit;
//            PowerControl.String_PowerData.MAXPower = ext_power_heat_data_0x0201.chassis_power_limit * 0.5f;
//        } else {
             // Fallback if referee data is not yet valid (optional, keep test_max or a safe default)
             PowerControl.Wheel_PowerData.MAXPower = test_max;
             PowerControl.String_PowerData.MAXPower = test_max * 0.5f;
//        }

        // 轮向电机功率计算 (DJI 3508)
        PowerControl.Wheel_PowerData.UpRLS(pid_vel_Wheel, toque_const_3508, rpm_to_rads_3508);
        
        // 舵向电机功率计算 (LK 4005)
        PowerControl.String_PowerData.UpRLS(pid_vel_String, toque_const_4005, rpm_to_rads_4005);

        // ========== 能量环 ==========
        // 获取裁判系统反馈的缓冲能量（单位：J，最大60J）
        float buffer_energy = (float)ext_power_heat_data_0x0202.chassis_power_buffer;

        // 能量环更新：以35J为阈值
        // buffer_energy > 35J → 能量充足，放电补充功率，提升 MAXPower
        // buffer_energy < 35J → 能量不足，充电超级电容，降低 MAXPower
        PowerControl.Wheel_PowerData.EnergyLoopUpdate(buffer_energy, test_max, dt);
        // 舵向电机功率按比例跟随轮向电机
        PowerControl.String_PowerData.MAXPower = PowerControl.Wheel_PowerData.MAXPower * 0.5f;

        float lim_cin_power = 100 - 1.0f;

        // 发送CAN指令
        BSP::SuperCap::cap.SetSendValue(lim_cin_power);
        BSP::SuperCap::cap.sendCAN(&hcan2, CAN_TX_MAILBOX0);
        // 功率验证模式 - 对比总估计功率 vs PM01 实测
        float total_est_power = PowerControl.Wheel_PowerData.EstimatedPower 
                              + PowerControl.String_PowerData.EstimatedPower;
        // Tools.vofaSend(total_est_power,                                   // 总估计功率
        //               BSP::Power::pm01.pm_power,                          // PM01 实测总功率
        //               PowerControl.Wheel_PowerData.EstimatedPower,        // 轮向估计功率
        //               PowerControl.String_PowerData.EstimatedPower,       // 舵向估计功率
        //               PowerControl.Wheel_PowerData.MAXPower,              // 功率上限
        //               total_est_power - BSP::Power::pm01.pm_power);       // 误差

        osDelay(1);
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
        // ⚠️ 功率是标量，需要取绝对值！
        // 否则不同方向的电机功率会互相抵消
        EffectivePower +=
            fabs(motor_interface_->GetCurrent(i+1) * motor_interface_->GetSpeed(i+1))  * rpm_to_rads;

        samples[0][0] += fabs(motor_interface_->GetSpeed(i+1)) * rpm_to_rads;
        samples[1][0] +=
            motor_interface_->GetCurrent(i+1) * motor_interface_->GetCurrent(i+1) * toque_const * toque_const;
    }
    //&& Dir_Event.getSuperCap() == false && Dir_Event.GetDir_String() == false
//    if (is_RLS == true) {
//        //        params = rls.update(samples, BSP::SuperCap::cap.getOutPower() - EffectivePower - k3);
//        params = rls.update(samples, BSP::Power::pm01.pm_power - EffectivePower - k3);

//        // }
//        k1 = fmax(params[0][0], 1e-5f);
//        k2 = fmax(params[1][0], 1e-5f);
//    }

    Cur_EstimatedPower = k1 * samples[0][0] + k2 * samples[1][0] + EffectivePower + k3;

    EstimatedPower = 0;
    for (int i = 0; i < 4; i++) {
        Initial_Est_power[i] = pid->GetCout() * toque_const * motor_interface_->GetSpeed(i+1) * rpm_to_rads
                               +fabs(motor_interface_->GetSpeed(i+1) * rpm_to_rads) * k1 +
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
    // 计算总误差和总原计划功率
    float sumErr = 0.0f;

    for (int i = 0; i < 4; i++) {
        sumErr += fabsf(pid[i].GetErr());
    }

    for (int i = 0; i < 4; i++) {
        pMaxPower[i] = MAXPower * (fabsf(pid[i].GetErr()) / sumErr);
        if (pMaxPower[i] < 0) {
            continue;
        }
    }
}

// 能量环实现
// 基于缓冲能量的PD控制器，动态调整功率上限 MAXPower
// energy_fb: 能量反馈值（裁判系统缓冲能量，单位J，范围0~60）
// ref_power: 用户设定的功率参考值（如 test_max）
// dt: 控制周期（秒）
void PowerUpData_t::EnergyLoopUpdate(float energy_fb, float ref_power, float dt)
{
    energy_feedback = energy_fb;
    P_ref = ref_power;

    // e(t) = sqrt(E_s) - sqrt(E_f)
    // 使用开方过渡函数：能量高时变化平缓，能量低时变化剧烈
    energy_err = sqrtf(fmaxf(energy_target, 0.0f)) - sqrtf(fmaxf(energy_feedback, 0.0f));

    // PD控制器: P_max = P_ref - Kp * e(t) - Kd * de/dt
    // 当 energy_fb > 35J 时，e(t) < 0，P_max > P_ref → 放电补充功率
    // 当 energy_fb < 35J 时，e(t) > 0，P_max < P_ref → 充电超级电容
    float de_dt = (dt > 1e-6f) ? (energy_err - energy_err_prev) / dt : 0.0f;
    energy_pmax_output = P_ref - energy_Kp * energy_err - energy_Kd * de_dt;

    // 功率上下限 clamp
    energy_pmax_output = fmaxf(energy_pmax_output, MIN_POWER);
    energy_pmax_output = fminf(energy_pmax_output, P_ref + 300.0f);  // 电容最大额外补偿300W

    // 更新误差历史
    energy_err_prev = energy_err;

    // 将能量环输出写入 MAXPower，供功率环使用
    MAXPower = energy_pmax_output;
}

// 计算应分配的力矩（功率环 - 未修改）
void PowerUpData_t::UpCalcMaxTorque(float *final_Out, PID *pid, const float toque_const, const float rpm_to_rads)
{
    if (EstimatedPower > MAXPower) 
    {
        for (int i = 0; i < 4; i++) {
            float omega = motor_interface_->GetSpeed(i+1) * rpm_to_rads;

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

            Cmd_MaxT[i] = Tools.clamp(Cmd_MaxT[i], 16384.0f, -16384.0f);

            final_Out[i] = Cmd_MaxT[i];
        }
    }
}
