
#include "ChassisTask.hpp"
#include "../APP/Referee/RM_RefereeSystem.h"
#include "../Task/CommunicationTask.hpp"
//#include "HAL.hpp"
#include "../APP/State.hpp"
#include "../APP/Variable.hpp"
#include "cmsis_os2.h"
#include "../Task/PowerTask.hpp"
#include "../APP/Remote/KeyBroad.hpp"
#include "../APP/Remote/Mode.hpp"
#include "../APP/UI/Static/darw_static.hpp"
#include "../APP/UI/UI_Queue.hpp"
#include "../BSP/Remote/Dbus.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../BSP/Motor/Lk/Lk_motor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../Algorithm/ChassisCalculation/StringWheel.hpp"
#include "../Task/CallBack.hpp"
#define p 3.14159
#define p3 2.35619
#define p2 1.570795
#define p4 0.7853975
TaskManager taskManager;
float Wheel_Azimuth[4] = {5 * My_PI / 4, 7 * My_PI / 4, My_PI / 4, 3 * My_PI / 4};
float phase[4] = {3.52173 + p3, 3.99659 + p4, 4.53233 - p4, 5.05887 - p3};
//3.52173  3.99659 4.53233 5.05887
float torque_ff[4] = {0.0f};
float pitch_deg = 0.0f;
float pitch_rad = 0.0f;
int16_t iqControl[4];	
float control_value;
float target_angle;


void ChassisTask(void *argument)
{
    osDelay(500);

    taskManager.addTask<Chassis_Task>();

    for (;;)
    {   
        taskManager.updateAll();
        osDelay(1);
    }
}
float k = 0.0f;
float tar_vw_angle = 3.1415926535f;
static constexpr float FOLLOW_YAW_DEADZONE_DEG = 1.0f;

//=== 状态处理器实现 ===//
class Chassis_Task::UniversalHandler : public StateHandler
{
    Chassis_Task &m_task;

  public:
    explicit UniversalHandler(Chassis_Task &task) : m_task(task)
    {
    }

    void UniversalTarget()
    {
        auto cos_theta = cosf(Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);
        auto sin_theta = sinf(Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);

        float vx_slope = ApplySlope(slope_vx, TAR_LX * 660, Chassis_Data.vx);
        float vy_slope = ApplySlope(slope_vy, TAR_LY * 660, Chassis_Data.vy);
        float vw_slope = ApplySlope(slope_vw, TAR_VW * 660, Chassis_Data.vw);

        Chassis_Data.vx = (vx_slope * cos_theta - vy_slope * sin_theta);
        Chassis_Data.vy = (vx_slope * sin_theta + vy_slope * cos_theta);
        Chassis_Data.vw = vw_slope;
    }

    void handle() override
    {
        // 可访问m_task的私有成员进行底盘操作
        UniversalTarget();
        m_task.Wheel_UpData();
        m_task.Filtering();
        m_task.PID_Updata();
        m_task.CAN_Setting();
        m_task.CAN_Send();
    }
};
float gyro_vel = 150;
class Chassis_Task::FollowHandler : public StateHandler
{
  public:
    Chassis_Task &m_task;

    explicit FollowHandler(Chassis_Task &task) : m_task(task)
    {
    }

    void FllowTarget()
    {
        float yaw_err = Gimbal_to_Chassis_Data.getEncoderAngleErr();
        if (fabsf(yaw_err) <= 2.0f * 0.017453f)
        {
            yaw_err = 0.0f;
        }

        auto cos_theta = cosf(yaw_err + tar_vw_angle);
        auto sin_theta = sinf(yaw_err + tar_vw_angle);
        
        pid_vw.GetPidPos(Kpid_vw, 0, yaw_err, 10000);

        float vx_slope = TAR_LX * 660.0f;                                                                                                                                                                         
        float vy_slope = TAR_LY * 660.0f;                                                                                                                                                                         
        float vw_slope = pid_vw.GetCout(); 
        
        
        
        Chassis_Data.vx = (vx_slope * cos_theta - vy_slope * sin_theta);
        Chassis_Data.vy = (vx_slope * sin_theta + vy_slope * cos_theta);
        Chassis_Data.vw = (vw_slope);

        td_FF_Tar.Calc(TAR_LX * 660);
			
    }

    void handle() override
    {
        // 可访问m_task的私有成员进行底盘操作
 
            FllowTarget();
            //m_task.applyRearBrake(0.1);
            m_task.Wheel_UpData();
            m_task.Filtering();
            m_task.PID_Updata();
            m_task.CAN_Setting();
            m_task.CAN_Send();
        
    }
};
float angle;
class Chassis_Task::KeyBoardHandler : public StateHandler
{
  public:
    Chassis_Task &m_task;

    explicit KeyBoardHandler(Chassis_Task &task) : m_task(task)
    {
    }

    void FllowTarget()
    {
        auto cos_theta = cosf(Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);
        auto sin_theta = sinf(Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);

        float vx_slope = TAR_LX * 660.0f;                                                                                                                                                                         
        float vy_slope = TAR_LY * 660.0f;                                                                                                                                                                         

        angle = Gimbal_to_Chassis_Data.getTargetOffsetAngle();

        float vw_target = 0.0f;
        if (Gimbal_to_Chassis_Data.getRotatingVel() > 0)
        {
            vw_target = Gimbal_to_Chassis_Data.getRotatingVel() * 4;
        }
        else
        {
            pid_vw.GetPidPos(Kpid_vw, 0, -Gimbal_to_Chassis_Data.getEncoderAngleErr(), 10000);
            vw_target = pid_vw.GetCout(); 
        }                                                                                                                                                                                                                                                                                                                                               
        float vw_slope = pid_vw.GetCout(); 

        if (Gimbal_to_Chassis_Data.getShitf())
        {
            Chassis_Data.now_power = 30.0f + ext_power_heat_data_0x0201.chassis_power_limit;
        }
        else
        {
            //Chassis_Data.now_power = ext_power_heat_data_0x0201.chassis_power_limit +  Gimbal_to_Chassis_Data.getPower() - 5;
            Chassis_Data.now_power = Tools.clamp(ext_power_heat_data_0x0201.chassis_power_limit 
                                        + Gimbal_to_Chassis_Data.getPower(), 120.0f,20) -5;
        }

        // if (BSP::Power::pm01.cout_voltage < 12.0f)
        // {
        //     Chassis_Data.now_power = ext_power_heat_data_0x0201.chassis_power_limit - 10.0f;
        // }

        //        PowerControl.setMaxPower(Chassis_Data.now_power);

        Chassis_Data.vx = (vx_slope * cos_theta - vy_slope * sin_theta);
        Chassis_Data.vy = (vx_slope * sin_theta + vy_slope * cos_theta);
        Chassis_Data.vw = vw_slope;

        td_FF_Tar.Calc(TAR_LX * 660);
    }

    void handle() override
    {
        // 可访问m_task的私有成员进行底盘操作

        FllowTarget();
        m_task.Wheel_UpData();
        m_task.Filtering();
        m_task.PID_Updata();
        m_task.CAN_Setting();
        m_task.CAN_Send();
    }
};
float ROTATION_BIAS = 0.00079999998f;
class Chassis_Task::RotatingHandler : public StateHandler
{
  public:
    Chassis_Task &m_task;

    explicit RotatingHandler(Chassis_Task &task) : m_task(task)
    {
    }

    void RotatingTarget()
    {
        auto cos_theta = cosf(Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle + ROTATION_BIAS * Chassis_Data.vw);
        auto sin_theta = sinf(Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle + ROTATION_BIAS * Chassis_Data.vw);

        // Use Get_Out() as feedback to create an open-loop slope in Gimbal Frame.
        // Using Chassis_Data.vx/vy (Chassis Frame) here would cause errors as they oscillate during rotation.
        float vx_slope = ApplySlope(slope_vx, TAR_LX * 660, slope_vx.Get_Out());
        float vy_slope = ApplySlope(slope_vy, TAR_LY * 660, slope_vy.Get_Out());
        float vw_slope = ApplySlope(slope_vw, TAR_VW * 660, Chassis_Data.vw);

        //        pid_vw.GetPidPos(Kpid_vw, 0, Gimbal_to_Chassis_Data.getEncoderAngleErr(), 10000);
        //        tar_vw.Calc(pid_vw.GetCout());

        Chassis_Data.vx = (vx_slope * cos_theta - vy_slope * sin_theta);
        Chassis_Data.vy = (vx_slope * sin_theta + vy_slope * cos_theta);
        Chassis_Data.vw = vw_slope;

        td_FF_Tar.Calc(TAR_LX * 660);
    }

    void handle() override
    {
        RotatingTarget();
        m_task.Wheel_UpData();
        m_task.Filtering();
        m_task.PID_Updata();
        m_task.CAN_Setting();
        m_task.CAN_Send();
    }
};

class Chassis_Task::StopHandler : public StateHandler
{
    Chassis_Task &m_task;

  public:
    explicit StopHandler(Chassis_Task &task) : m_task(task)
    {
    }

    void handle() override
    {
        // 执行急停相关操作
        // Base_UpData();
        m_task.Tar_Updata();    
        m_task.Wheel_UpData();                           
        m_task.PID_Updata();
        
        //PID_Updata();

        pid_vel_String[0].clearPID();
        pid_vel_String[1].clearPID();
        pid_vel_String[2].clearPID();
        pid_vel_String[3].clearPID();

        pid_vel_Wheel[0].clearPID();
        pid_vel_Wheel[1].clearPID();
        pid_vel_Wheel[2].clearPID();
        pid_vel_Wheel[3].clearPID();

        m_task.CAN_Setting();
        m_task.CAN_Send();
    }
};


//=== 任务方法实现 ===//
Chassis_Task::Chassis_Task()
: slope_speed{
          Class_Slope(2, 3, Slope_First_REAL),
          Class_Slope(2, 3, Slope_First_REAL),
          Class_Slope(2, 3, Slope_First_REAL),
          Class_Slope(2, 3, Slope_First_REAL)},
          stringIk(0.22f, 0.09f, Wheel_Azimuth, phase) 
{
    // 初始化默认状态
    updateState();
}

void Chassis_Task::executeState()
{
    if (m_stateHandler)
    {
        m_stateHandler->handle();
    }
}

uint8_t state_num;
void Chassis_Task::updateState()
{
    using namespace BSP::Remote;

    auto switch_right = dr16.switchRight();
    auto switch_left = dr16.switchLeft();

    if(Mode::Chassis::Move()){
        m_currentState = State::MoveState;
    } else if (Mode::Chassis::Universal()) {
        m_currentState = State::UniversalState;
    } else if (Mode::Chassis::Follow()) {
        m_currentState = State::FollowState;
    } else if (Mode::Chassis::Rotating()) {
        m_currentState = State::RotatingState;
    } else if (Mode::Chassis::KeyBoard()) {
        m_currentState = State::KeyBoardState;
    } else if (Mode::Chassis::Stop()) {
        m_currentState = State::StopState;

    }


    // 更新状态处理器
    switch (m_currentState)
    {
    case State::UniversalState:
        m_stateHandler = std::make_unique<UniversalHandler>(*this);
        break;
    case State::FollowState:
        m_stateHandler = std::make_unique<FollowHandler>(*this);
        break;
    case State::RotatingState:
        m_stateHandler = std::make_unique<RotatingHandler>(*this);
        break;
    case State::KeyBoardState:
        m_stateHandler = std::make_unique<KeyBoardHandler>(*this);
        break;
    case State::StopState:
        m_stateHandler = std::make_unique<StopHandler>(*this);
        break;
    }
    
}

// 将期望值做滤波后传入轮子
void Chassis_Task::Tar_Updata()
{
    tar_vx.Calc(TAR_LX * 660);
    tar_vy.Calc(TAR_LY * 660);
    tar_vw.Calc(TAR_RX * 660);

    Chassis_Data.vx = tar_vx.x1;
    Chassis_Data.vy = tar_vy.x1;
    Chassis_Data.vw = tar_vw.x1;

    td_FF_Tar.Calc(TAR_LX * 660);
}

float sin_t;
uint32_t ms;
float hz;
bool is_sin;
uint16_t pos;
float ude_tar;

// 将运动学解算相关，并对速度与角度进行过零处理
void Chassis_Task::Wheel_UpData()
{   
    
    for (int i = 0; i < 4; ++i)
    {
        slope_speed[i].TIM_Calculate_PeriodElapsedCallback(); // 每次都更新
    }

    for (int i = 0; i < 4; i++) {
        float currentAngle = BSP::Motor::LK::Motor4005.getAddAngleRad(i+1);
        stringIk.Set_current_steer_angles(currentAngle, i);
    }
    // 对轮子进行运动学变换
    stringIk.StringInvKinematics(Chassis_Data.vx / 660.0f, Chassis_Data.vy / 660.0f, Chassis_Data.vw / 660.0f, 0.0f, 15.0f, 30.0f);

    // 储存最小角判断的速度
    for (int i = 0; i < 4; i++)
    {
        Chassis_Data.tar_speed[i] = stringIk.GetMotor_wheel(i);
    }   

    // 储存最小角判断的角度，并将其转换为0-360度范围
    for (int i = 0; i < 4; i++)
    {
        Chassis_Data.tar_angle[i] = stringIk.GetMotor_direction(i) * 57.2957795; 
    }

    // 调整初始角度偏移，确保角度一致
    Chassis_Data.getMinPos[0] =
        Tools.MinPosHelm(Chassis_Data.tar_angle[0], BSP::Motor::LK::Motor4005.getAngleDeg(1), 
                        &Chassis_Data.tar_speed[0], 1000, 360);
    Chassis_Data.getMinPos[1] =
        Tools.MinPosHelm(Chassis_Data.tar_angle[1], BSP::Motor::LK::Motor4005.getAngleDeg(2),
                        &Chassis_Data.tar_speed[1], 1000, 360);
    Chassis_Data.getMinPos[2] =
        Tools.MinPosHelm(Chassis_Data.tar_angle[2], BSP::Motor::LK::Motor4005.getAngleDeg(3), 
                        &Chassis_Data.tar_speed[2], 1000, 360);
    Chassis_Data.getMinPos[3] =
        Tools.MinPosHelm(Chassis_Data.tar_angle[3], BSP::Motor::LK::Motor4005.getAngleDeg(4), 
                        &Chassis_Data.tar_speed[3], 1000, 360);

    if (is_sin == true)
    {
        sin_t = 4096 + sinf(2 * 3.1415926 * ms * 0.001 * hz) * 4000;
        ude_tar = 4096 + sinf(2 * 3.1415926 * ms * 0.001 * hz) * 4000;
        ms++;
    }
    else
        sin_t = pos;
    Chassis_Data.Zero_cross[0] = Tools.Zero_crossing_processing(
        Chassis_Data.getMinPos[0], BSP::Motor::LK::Motor4005.getAngleDeg(1), 360);
    Chassis_Data.Zero_cross[1] = Tools.Zero_crossing_processing(
        Chassis_Data.getMinPos[1], BSP::Motor::LK::Motor4005.getAngleDeg(2), 360);
    Chassis_Data.Zero_cross[2] = Tools.Zero_crossing_processing(
        Chassis_Data.getMinPos[2], BSP::Motor::LK::Motor4005.getAngleDeg(3), 360);
    Chassis_Data.Zero_cross[3] = Tools.Zero_crossing_processing(
        Chassis_Data.getMinPos[3], BSP::Motor::LK::Motor4005.getAngleDeg(4), 360);
}

// 滤波器更新
void Chassis_Task::Filtering()
{
    // 电机一般速度反馈噪声大
    for (int i = 0; i < 4; i++)
    {
        td_3508_speed[i].Calc(BSP::Motor::Dji::Motor3508.getVelocityRads(i+1));
    }
}

// 舵向电机目标角度死区阈值（度）
static constexpr float STEER_ANGLE_DEADZONE = 2.0f;
// 上一次的目标角度（静态变量，保持状态）
static float last_steer_target[4] = {0.0f, 0.0f, 0.0f, 0.0f};
// 初始化标志
static bool steer_target_initialized = false;

void Chassis_Task::PID_Updata()
{   
    // 初始化上一次目标角度（首次运行时）
    if (!steer_target_initialized)
    {
        for (int i = 0; i < 4; i++)
        {
            last_steer_target[i] = BSP::Motor::LK::Motor4005.getAngleDeg(i+1);
        }
        steer_target_initialized = true;
    }
    
    // 舵向电机更新
    for (int i = 0; i < 4; i++)
    {
        // 舵向电机前馈更新
        feed_4005[i].UpData(Chassis_Data.FF_Zero_cross[i]);
        Chassis_Data.FF_Zero_cross[i] = Tools.Round_Error(feed_4005[i].cout, feed_4005[i].target_e, 360);

        // 目标角度死区处理：当目标角度变化很小时，保持上一次的目标
        float current_target = Chassis_Data.Zero_cross[i];
        float angle_diff = fabsf(current_target - last_steer_target[i]);
        
        // 处理跨越0/360度的情况
        if (angle_diff > 180.0f)
        {
            angle_diff = 360.0f - angle_diff;
        }
        
        // 如果变化超过死区阈值，更新目标；否则保持上一次的目标
        if (angle_diff > STEER_ANGLE_DEADZONE)
        {
            last_steer_target[i] = current_target;
        }
        
        pid_angle_String[i].GetPidPos(Kpid_4005_angle, last_steer_target[i], BSP::Motor::LK::Motor4005.getAngleDeg(i+1), 2048);
        
        // 舵向电机速度环更新
        pid_vel_String[i].GetPidPos(Kpid_4005_vel, pid_angle_String[i].pid.cout, BSP::Motor::LK::Motor4005.getVelocityRpm(i+1), 2048);
        
    }

    // 轮毂电机速度环更新
    for (int i = 0; i < 4; i++)
    {
        pid_vel_Wheel[i].GetPidPos(Kpid_3508_vel, Chassis_Data.tar_speed[i], td_3508_speed[i].x1, 16384.0f);
    }
}

void Chassis_Task::CAN_Setting()
{
		::state_num = state_num;
        bool is_ude = true;
    // for (int i = 0; i < 4; i++)
    // {
  
    //     Chassis_Data.final_4005_Out[i] = pid_vel_String[i].GetCout();
    //     control_value = Chassis_Data.final_4005_Out[i];
    //     iqControl[i] = (int16_t)control_value;
    //     BSP::Motor::LK::Motor4005.MultControl(&hcan1, iqControl);
    // }
    for (int i = 0; i < 4; i++) {
        Chassis_Data.final_3508_Out[i] = pid_vel_Wheel[i].GetCout();
        Chassis_Data.final_4005_Out[i] = pid_vel_String[i].GetCout();
    }

    // ==================== 新功率控制算法 (基于功率控制.md) ====================
    // 使用六参数模型 + 衰减电流法
    // 舵向电机优先分配50%功率，轮向使用剩余功率
   // ApplyPowerLimit(Chassis_Data.final_3508_Out, Chassis_Data.final_4005_Out);
    // ========================================================================

   // ==================== 动态功率分配策略 ====================
   float total_power_limit = PowerControl.Wheel_PowerData.MAXPower; // 使用设定的总限制 (例如 40W)
   
   // 1. 获取舵向估计功率（未限制前）
   // 注意：此时 String_PowerData.EstimatedPower 是基于当前指令算出的需求功率
   float string_est_power = PowerControl.String_PowerData.EstimatedPower;
   
   // 2. 舵向优先分配
   // 规则：给舵向分配它需要的功率，但最大不超过总功率的 40% (保证轮子至少有60%动力)
   float string_max_limit = total_power_limit * 0.4f;
   float string_alloc_power = string_est_power;
   if (string_alloc_power > string_max_limit) string_alloc_power = string_max_limit;
   // 保证舵向至少有 10W 可用 (防止太低无法转向)
   if (string_alloc_power < 10.0f) string_alloc_power = 10.0f; 

   // 3. 轮向分配剩余功率
   float wheel_alloc_power = total_power_limit - string_alloc_power;
   if (wheel_alloc_power < 0) wheel_alloc_power = 0;

   // 4. 应用限制
   PowerControl.String_PowerData.MAXPower = string_alloc_power;
   PowerControl.Wheel_PowerData.MAXPower  = wheel_alloc_power;

   // 执行限制计算
   PowerControl.String_PowerData.UpScaleMaxPow(pid_vel_String);
   PowerControl.String_PowerData.UpCalcMaxTorque(Chassis_Data.final_4005_Out, pid_vel_String,
                                                 toque_const_4005, rpm_to_rads_4005);                                          

   PowerControl.Wheel_PowerData.UpScaleMaxPow(pid_vel_Wheel);
   PowerControl.Wheel_PowerData.UpCalcMaxTorque(Chassis_Data.final_3508_Out, pid_vel_Wheel,
                                                toque_const_3508, rpm_to_rads_3508);
                                                
   // 恢复 MAXPower 为默认值 (防止影响下一次循环的逻辑，虽然这里每帧都重算)
   PowerControl.Wheel_PowerData.MAXPower = total_power_limit; 
   // ============================================================

    // 轮向电机输出
    for(int i = 0; i < 4; i++)
    {
        BSP::Motor::Dji::Motor3508.setCAN(Chassis_Data.final_3508_Out[i], (i + 1));
    }
}


void Chassis_Task::CAN_Send()
{   

    // 发送数据
    // if(BSP::Remote::dr16.isDrOnline() == false)
    // {
    //     for(int i = 0; i < 4; i++)
    //     {
    //         BSP::Motor::Dji::Motor3508.setCAN(0, (i + 1));
    //         BSP::Motor::LK::Motor4005.ctrl_Multi(iqControl);
    //     }

    // }

    if (Send_ms == 1)
    {
        static uint8_t status_count = 0;
        status_count++;
        if (status_count > 10) status_count = 1;
        
        if (status_count <= 4)
        {
            BSP::Motor::LK::Motor4005.ReadStatus1(status_count);
        }

        for (int i = 0; i < 4; i++)
        {
            Chassis_Data.final_4005_Out[i] = pid_vel_String[i].GetCout();
            iqControl[i] = (int16_t)Chassis_Data.final_4005_Out[i];
        }
        BSP::Motor::LK::Motor4005.ctrl_Multi(iqControl);
    }
    if (Send_ms == 0)
    {
        // 轮向电机发送
        BSP::Motor::Dji::Motor3508.sendCAN();
    }

    Send_ms ++;         
    Send_ms %= 2;  

   Tools.vofaSend(BSP::Motor::LK::Motor4005.getAngleRad(1), BSP::Motor::LK::Motor4005.getAngleRad(2), 
                    BSP::Motor::LK::Motor4005.getAngleRad(3), BSP::Motor::LK::Motor4005.getAngleRad(4), 0, 0);
}   



