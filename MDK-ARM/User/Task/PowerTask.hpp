#pragma once

#include "../Algorithm/RLS.hpp"
#include "../APP/Variable.hpp"
#include "arm_math.h"

#define My_PI 3.14152653529799323

// 扭矩常数定义
#define toque_const_3508 0.00036621f
#define rpm_to_rads_3508 0.00664267f

#define toque_const_4005 0.000117187f
#define rpm_to_rads_4005 0.010471966f

#define pMAX 120.0f

// 前向声明
namespace BSP::Motor::Dji {
    template<uint8_t N> class GM3508;
}
namespace BSP::Motor::LK {
    template<uint8_t N> class LK4005;
}

namespace SGPowerControl
{
    struct PowerObj {
    public:
        float pidOutput;    // torque current command, [-maxOutput, maxOutput], no unit
        float curAv;        // Measured angular velocity, [-maxAv, maxAv], rad/s
        float setAv;        // target angular velocity, [-maxAv, maxAv], rad/s
        float pidMaxOutput; // pid max output
    };

    // 电机数据获取接口
    class IMotorInterface
    {
    public:
        virtual ~IMotorInterface() = default;
        virtual float GetCurrent(uint8_t index) const = 0; 
        virtual float GetSpeed(uint8_t index) const = 0;
        virtual float GetMaxCurrent() const = 0;
    };

    // DJI电机适配器
    class DjiMotorAdapter : public IMotorInterface
    {
    private:
        BSP::Motor::Dji::GM3508<4>& motor_;
    public:
        DjiMotorAdapter(BSP::Motor::Dji::GM3508<4>& motor) : motor_(motor) {}
        
        float GetCurrent(uint8_t index) const override {
            return motor_.getCurrent(index);
        }
        
        float GetSpeed(uint8_t index) const override {
            return motor_.getVelocityRads(index);
        }
        float GetMaxCurrent() const override 
        {
            return 16384.0f;       
        }

    };

    // LK电机适配器
    class LkMotorAdapter : public IMotorInterface
    {
    private:
        BSP::Motor::LK::LK4005<4>& motor_;
    public:
        LkMotorAdapter(BSP::Motor::LK::LK4005<4>& motor) : motor_(motor) {}
        float GetCurrent(uint8_t index) const override {
            return motor_.getCurrent(index);
        }
        
        float GetSpeed(uint8_t index) const override {
            return motor_.getVelocityRads(index);
        }
        float GetMaxCurrent() const override 
        {
            return 2048.0f;
        }      
    };

    class PowerUpData_t
    {
    private:
        IMotorInterface* motor_interface_;
        
    public:
        Math::RLS<2> rls;

        Matrixf<2, 1> samples;
        Matrixf<2, 1> params;

        float MAXPower;
        PowerUpData_t() : rls(1e-4f, 0.99995f), motor_interface_(nullptr), Init_flag(false)
        {
            // 初始化成员变量
            k1 = k2 = k3 = k0 = 0.0f;
            Energy = 0.0f;
            EstimatedPower = 0.0f;
            Cur_EstimatedPower = 0.0f;
            EffectivePower = 0.0f;
            E_lower = 0.0f;
            E_upper = 0.0f;

        }

        ~PowerUpData_t() {
            if (motor_interface_) {
                delete motor_interface_;
            }
        }

        bool is_RLS = false;

        /* data */
        float k1, k2, k3, k0;

        float Energy;

        float EstimatedPower;
        float Cur_EstimatedPower;

        float Initial_Est_power[4];

        float EffectivePower;

        float pMaxPower[4];
        double Cmd_MaxT[4];
		
		bool Init_flag;

        // 定义阈值参数
        float E_lower;
        float E_upper;

        // 设置电机接口
        void SetMotorInterface(IMotorInterface* interface) {
            delete motor_interface_;
            motor_interface_ = interface;
        }

        // 统一的功率计算方法
        void UpRLS(PID *pid, const float toque_const, const float rpm_to_rads);
    
        // 等比缩放的最大分配功率
        void UpScaleMaxPow(PID *pid);
    
        // 计算应分配的力矩
        void UpCalcMaxTorque(float *final_Out, PID *pid, const float toque_const, const float rpm_to_rads);

        // ========== 能量环参数 ==========
        float energy_Kp = 5.0f;            // 能量环P系数
        float energy_Kd = 0.0f;            // 能量环D系数（建议初始设为0，避免震荡）
        float energy_target = 35.0f;       // 能量目标值（J），缓冲能量阈值
        float energy_feedback = 0.0f;      // 能量反馈值
        float energy_err = 0.0f;           // 当前能量误差 e(t)
        float energy_err_prev = 0.0f;      // 上一次能量误差 e(t-1)
        float energy_pmax_output = 0.0f;   // 能量环输出的 P_max
        float P_ref = 60.0f;              // 用户设定的功率参考值
        static constexpr float MIN_POWER = 15.0f;          // 功率下限
        static constexpr float MAX_BUFFER_ENERGY = 60.0f;  // 缓冲能量上限（J）

        // 能量环更新方法
        void EnergyLoopUpdate(float energy_fb, float ref_power, float dt);
    };

    class PowerTask_t
    {
    public:
        PowerTask_t()
        {
            SetDefaultConfig();
        }
        void SetDefaultConfig() {
            // 轮向电机配置 (DJI 3508)
            // 优化参数：提高功率利用率
            Wheel_PowerData.MAXPower     = 40.0f;
            Wheel_PowerData.k1           = 3.8f;    // 回调：从 8.0 -> 5.5
            Wheel_PowerData.k2           = 1.2f;    // 回调：从 3.0 -> 1.5
            Wheel_PowerData.k3           = 4.0f;
            Wheel_PowerData.is_RLS       = false;
            Wheel_PowerData.E_upper      = 1000.0f;
            Wheel_PowerData.E_lower      = 500.0f;

            // 舵向电机配置 (LK 4005)
            String_PowerData.MAXPower    = 40.0f * 0.6f;
            String_PowerData.k1          = 25.0f;   // 回调：从 25.76 -> 16.0
            String_PowerData.k2          = 5.5f;    // 大幅回调：从 5.0 -> 0.5
            String_PowerData.k3          = 5.0f;
            String_PowerData.is_RLS      = false;    
            String_PowerData.E_upper     = 500.0f;
            String_PowerData.E_lower     = 100.0f;
        }

        // 初始化电机接口
        void InitMotorInterfaces(BSP::Motor::Dji::GM3508<4>& wheel_motor, BSP::Motor::LK::LK4005<4>& string_motor) {
            Wheel_PowerData.SetMotorInterface(new DjiMotorAdapter(wheel_motor));
            String_PowerData.SetMotorInterface(new LkMotorAdapter(string_motor));
        }

        PowerUpData_t String_PowerData;
        PowerUpData_t Wheel_PowerData;

        inline float GetEstWheelPow() const
        {
            return Wheel_PowerData.EstimatedPower;
        }

        inline float GetEstStringPow() const
        {
            return String_PowerData.EstimatedPower;
        }

        inline void setMaxPower(float maxPower)
        {
            Wheel_PowerData.MAXPower  = maxPower;
            String_PowerData.MAXPower = maxPower * 0.6f;
        }

        inline uint16_t getMAXPower() const
        {
            return static_cast<uint16_t>(Wheel_PowerData.MAXPower);
        }
        

    };
} // namespace SGPowerControl

static inline bool floatEqual(float a, float b)
{
    return fabs(a - b) < 1e-5f;
}

extern SGPowerControl::PowerTask_t PowerControl;

#ifdef __cplusplus
extern "C" {
#endif

void RLSTask(void *argument);

#ifdef __cplusplus
}
#endif