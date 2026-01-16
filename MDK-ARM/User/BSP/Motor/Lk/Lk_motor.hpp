#ifndef Lk_Motor_hpp
#define Lk_Motor_hpp
#pragma once

#include "../BSP/Motor/MotorBase.hpp"
#include "../HAL/CAN/can_hal.hpp"

static double deg_to_rad = 0.017453292519611;
static double rad_to_deg = 1 / 0.017453292519611;

namespace BSP::Motor::LK
{

// 参数结构体定义
struct Parameters
{
    double reduction_ratio;      // 减速比
    double torque_constant;      // 力矩常数 (Nm/A)
    double feedback_current_max; // 反馈最大电流 (A)
    double current_max;          // 最大电流 (A)
    double encoder_resolution;   // 编码器分辨率

    // 自动计算的参数
    double encoder_to_deg;                  // 编码器值转角度系数
    double encoder_to_rpm;
    double rpm_to_radps;                    // RPM转角速度系数
    double current_to_torque_coefficient;   // 电流转扭矩系数   
    double feedback_to_current_coefficient; // 反馈电流转电流系数
    double deg_to_real;                     // 角度转实际角度系数



    // 构造函数带参数计算
    Parameters(double rr, double tc, double fmc, double mc, double er)
        : reduction_ratio(rr), torque_constant(tc), feedback_current_max(fmc), 
          current_max(mc), encoder_resolution(er)
    {
        encoder_to_deg = 360.0 / encoder_resolution;
        rpm_to_radps = 1 / reduction_ratio / 60 * 2 * PI;
        encoder_to_rpm = 1 / reduction_ratio;
        current_to_torque_coefficient = torque_constant / feedback_current_max * current_max;
        feedback_to_current_coefficient = current_max / feedback_current_max;
        deg_to_real = 1 / reduction_ratio;
    }
};

/**
 * @brief LK电机基类
 */
template <uint8_t N> class LkMotorBase : public MotorBase<N>
{
protected:
    struct LkMotorFeedback
    {
        uint8_t cmd;
        uint8_t temperature;
        int16_t current;
        int16_t velocity;
        uint16_t angle;
    };

    struct MultiAngleData
    {
        double total_angle;
        double last_angle;
        bool allow_accumulate;
        bool is_initialized;
    };

    /**
     * @brief 构造函数
     */
    LkMotorBase(uint16_t Init_id, const uint8_t (&recv_ids)[N], const uint32_t (&send_ids)[N], Parameters params)
        : init_address(Init_id), params_(params)
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            recv_idxs_[i] = recv_ids[i];
            send_idxs_[i] = send_ids[i];
        }

        // 初始化电机数据
        for (uint8_t i = 0; i < N; ++i)
        {
            multi_angle_data_[i].total_angle = 0.0;
            multi_angle_data_[i].last_angle = 0.0;
            multi_angle_data_[i].allow_accumulate = false;
            multi_angle_data_[i].is_initialized = false;
        }
    }

private:
    /**
     * @brief 将反馈数据转换为国际单位
     */
    void Configure(size_t i, const LkMotorFeedback& feedback)
    {
        const auto &params = params_;

        this->unit_data_[i].angle_Deg = feedback.angle * params.encoder_to_deg;
        this->unit_data_[i].angle_Rad = this->unit_data_[i].angle_Deg * params.deg_to_real;
        this->unit_data_[i].velocity_Rad = feedback.velocity * params.rpm_to_radps;
        this->unit_data_[i].velocity_Rpm = feedback.velocity * params.encoder_to_rpm;
        this->unit_data_[i].current_A = feedback.current * params.feedback_to_current_coefficient;
        this->unit_data_[i].torque_Nm = feedback.current * params.current_to_torque_coefficient;
        this->unit_data_[i].temperature_C = feedback.temperature;

        // 多圈角度计算
        if (multi_angle_data_[i].allow_accumulate) 
        {
            if (!multi_angle_data_[i].is_initialized)
            {
                multi_angle_data_[i].last_angle = this->unit_data_[i].angle_Deg;
                multi_angle_data_[i].is_initialized = true;
            }
            else
            {
                double last_angle = multi_angle_data_[i].last_angle;
                double delta = this->unit_data_[i].angle_Deg - last_angle;               
                // 处理360°跳变
                if (delta > 180.0) 
                    delta -= 360.0;
                else if (delta < -180.0) 
                    delta += 360.0;
                
                multi_angle_data_[i].total_angle += delta;
                this->unit_data_[i].add_angle = delta;
            }
        }
        multi_angle_data_[i].last_angle = this->unit_data_[i].angle_Deg;
        this->unit_data_[i].last_angle = this->unit_data_[i].angle_Deg;
    }

public:
    /**
     * @brief 解析CAN数据
     */
    void Parse(const HAL::CAN::Frame &frame) override
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            if (frame.id == init_address + recv_idxs_[i])
            {
                const uint8_t* pData = frame.data;
                
                feedback_[i].cmd = pData[0];
                feedback_[i].temperature = pData[1];               
                feedback_[i].current = (int16_t)((pData[3] << 8) | pData[2]);
                feedback_[i].velocity = (int16_t)((pData[5] << 8) | pData[4]);
                feedback_[i].angle = (uint16_t)((pData[7] << 8) | pData[6]);

                Configure(i, feedback_[i]);
                // 更新时间戳用于断联检测
                this->state_watch_[i].UpdateLastTime();
            }
        }
    }

    /**
     * @brief 使能LK电机
     */
    void On(uint8_t id)
    {
        uint8_t send_data[8] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        
        HAL::CAN::Frame frame;
        frame.id = init_address + send_idxs_[id - 1];
        frame.dlc = 8;
        memcpy(frame.data, send_data, sizeof(send_data));
        frame.is_extended_id = false;
        frame.is_remote_frame = false;
        
        HAL::CAN::get_can_bus_instance().get_can1().send(frame);
    }

    /**
     * @brief 失能LK电机
     */
    void Off(uint8_t id)
    {
        uint8_t send_data[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        
        HAL::CAN::Frame frame;
        frame.id = init_address + send_idxs_[id - 1];
        frame.dlc = 8;
        memcpy(frame.data, send_data, sizeof(send_data));
        frame.is_extended_id = false;
        frame.is_remote_frame = false;
        
        HAL::CAN::get_can_bus_instance().get_can1().send(frame);
    }

    /**
     * @brief 清除LK电机错误
     */
    void ClearErr(uint8_t id)
    {
        uint8_t send_data[8] = {0x9B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        
        HAL::CAN::Frame frame;
        frame.id = init_address + send_idxs_[id - 1];
        frame.dlc = 8;
        memcpy(frame.data, send_data, sizeof(send_data));
        frame.is_extended_id = false;
        frame.is_remote_frame = false;
        
        HAL::CAN::get_can_bus_instance().get_can1().send(frame);
    }

    /**
     * @brief LK电机扭矩控制
     */
    void ctrl_Torque(uint8_t id, int16_t torque)
    {
        if (torque > 2048) torque = 2048;
        if (torque < -2048) torque = -2048;
        
        uint8_t send_data[8];
        send_data[0] = 0xA1;
        send_data[1] = 0x00;
        send_data[2] = 0x00;
        send_data[3] = 0x00;
        send_data[4] = torque & 0xFF;
        send_data[5] = (torque >> 8) & 0xFF;
        send_data[6] = 0x00;
        send_data[7] = 0x00;

        HAL::CAN::Frame frame;
        frame.id = init_address + send_idxs_[id - 1];
        frame.dlc = 8;
        memcpy(frame.data, send_data, sizeof(send_data));
        frame.is_extended_id = false;
        frame.is_remote_frame = false;
        
        HAL::CAN::get_can_bus_instance().get_can1().send(frame);
    }

    /**
     * @brief LK电机位置速度控制
     */
    void ctrl_Position(uint8_t id, int32_t angle, uint16_t speed)
    {
        uint32_t encoder_value = angle * 100;
        
        uint8_t send_data[8];
        send_data[0] = 0xA4;
        send_data[1] = 0x00;
        send_data[2] = speed & 0xFF;
        send_data[3] = (speed >> 8) & 0xFF;
        send_data[4] = encoder_value & 0xFF;
        send_data[5] = (encoder_value >> 8) & 0xFF;
        send_data[6] = (encoder_value >> 16) & 0xFF;
        send_data[7] = (encoder_value >> 24) & 0xFF;

        HAL::CAN::Frame frame;
        frame.id = init_address + send_idxs_[id - 1];
        frame.dlc = 8;
        memcpy(frame.data, send_data, sizeof(send_data));
        frame.is_extended_id = false;
        frame.is_remote_frame = false;
        
        HAL::CAN::get_can_bus_instance().get_can1().send(frame);
    }

    /**
     * @brief LK电机多电机控制（广播模式）
     */
    void ctrl_Multi(const int16_t iqControl[4])
    {
        auto& can1 = HAL::CAN::get_can_bus_instance().get_can1();
        if (HAL_CAN_GetTxMailboxesFreeLevel(can1.get_handle()) == 0)
        {
            return;  // 邮箱满，跳过本次发送
        }
        uint8_t send_data[8];
        for (int i = 0; i < 4; ++i) 
        {
            send_data[i * 2] = iqControl[i] & 0xFF;           
            send_data[i * 2 + 1] = (iqControl[i] >> 8) & 0xFF; 
        }

        HAL::CAN::Frame frame;
        frame.id = 0x280;
        frame.dlc = 8;
        memcpy(frame.data, send_data, sizeof(send_data));
        frame.is_extended_id = false;
        frame.is_remote_frame = false;
        
        HAL::CAN::get_can_bus_instance().get_can1().send(frame);
    }

    /**
     * @brief 获取多圈角度
     */
    float getMultiAngle(uint8_t id)
    {
        return multi_angle_data_[id - 1].total_angle;
    }

    /**
     * @brief 获取电机的原始反馈角度
     */
    uint16_t getRawAngle(uint8_t id)
    {
        return feedback_[id - 1].angle;
    }

    /**
     * @brief 设置是否允许累计多圈角度
     */
    void setAllowAccumulate(uint8_t id, bool allow)
    {
        multi_angle_data_[id - 1].allow_accumulate = allow;
    }

    /**
     * @brief 获取是否允许累计多圈角度
     */
    bool getAllowAccumulate(uint8_t id)
    {
        return multi_angle_data_[id - 1].allow_accumulate;
    }

protected:
    const uint16_t init_address;
    uint8_t recv_idxs_[N];
    uint32_t send_idxs_[N];
    LkMotorFeedback feedback_[N];
    Parameters params_;
    MultiAngleData multi_angle_data_[N];
};

/**
 * @brief LK4005电机类
 */
template <uint8_t N> class LK4005 : public LkMotorBase<N>
{
public:
    LK4005(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs)[N])
        : LkMotorBase<N>(Init_id, ids, send_idxs,
                         Parameters(10.0,   // 减速比
                                   0.06,    // 扭矩常数
                                   2048,    // 最大反馈电流
                                   4,       // 最大电流 
                                   65536.0)) // 编码器分辨率
    {}
};

/**
 * @brief 电机实例
 */
inline LK4005<4> Motor4005{0x140, {1, 2, 3, 4}, {1, 2, 3, 4}};

} // namespace BSP::Motor::LK

#endif
