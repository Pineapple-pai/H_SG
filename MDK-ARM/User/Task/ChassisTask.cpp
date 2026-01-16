
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

TaskManager taskManager;
float Wheel_Azimuth[4] = {5 * My_PI / 4, 7 * My_PI / 4, 3 * My_PI / 4, My_PI / 4};
float phase[4] = {2.90079 + 3.14159, 6.352207 + 3.14159, 1.83559 + 3.14159, 5.44465 + 3.14159};

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
        auto cos_theta = cosf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);
        auto sin_theta = sinf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);

        float vx_slope = ApplySlope(slope_vx, TAR_LX * 660, Chassis_Data.vx);
        float vy_slope = ApplySlope(slope_vy, TAR_LY * 660, Chassis_Data.vy);
        float vw_slope = ApplySlope(slope_vw, pid_vw.GetCout(), Chassis_Data.vw);

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

        auto cos_theta = cosf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);
        auto sin_theta = sinf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);
        
        pid_vw.GetPidPos(Kpid_vw, 0, -Gimbal_to_Chassis_Data.getEncoderAngleErr(), 10000);
        float vx_slope = ApplySlope(slope_vx, TAR_LX * 660, Chassis_Data.vx);
        float vy_slope = ApplySlope(slope_vy, TAR_LY * 660, Chassis_Data.vy);
        float vw_slope = ApplySlope(slope_vw, pid_vw.GetCout(), Chassis_Data.vw);
        
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
        auto cos_theta = cosf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);
        auto sin_theta = sinf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);

        float vx_slope = ApplySlope(slope_vx, TAR_LX * 660, Chassis_Data.vx);
        float vy_slope = ApplySlope(slope_vy, TAR_LY * 660, Chassis_Data.vy);
        float vw_slope = ApplySlope(slope_vw, pid_vw.GetCout(), Chassis_Data.vw);

        angle = Gimbal_to_Chassis_Data.getTargetOffsetAngle();

        if (Gimbal_to_Chassis_Data.getRotatingVel() > 0)
        {
            tar_vw.Calc(Gimbal_to_Chassis_Data.getRotatingVel() * 4);
        }
        else
        {
            pid_vw.GetPidPos(Kpid_vw, 0, Gimbal_to_Chassis_Data.getEncoderAngleErr(), 10000);
            tar_vw.Calc(pid_vw.GetCout());
        }

        if (Gimbal_to_Chassis_Data.getShitf())
        {
            Chassis_Data.now_power = 30.0f + ext_power_heat_data_0x0201.chassis_power_limit;
        }
        else
        {
            //            Chassis_Data.now_power = ext_power_heat_data_0x0201.chassis_power_limit +
            //            Gimbal_to_Chassis_Data.getPower() - 5;

            Chassis_Data.now_power =
                Tools.clamp(ext_power_heat_data_0x0201.chassis_power_limit + Gimbal_to_Chassis_Data.getPower(), 120.0f,
                            20) -
                5;
        }

        if (BSP::Power::pm01.cout_voltage < 12.0f)
        {
            Chassis_Data.now_power = ext_power_heat_data_0x0201.chassis_power_limit - 10.0f;
        }

        //        PowerControl.setMaxPower(Chassis_Data.now_power);

        if (Gimbal_to_Chassis_Data.getF5())
        {
            UI::UI_send_queue.is_Delete_all = true;
            UI::Static::UI_static.Init();
        }
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
        auto cos_theta = cosf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle + ROTATION_BIAS * Chassis_Data.vw);
        auto sin_theta = sinf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle + ROTATION_BIAS * Chassis_Data.vw);

        float vx_slope = ApplySlope(slope_vx, TAR_LX * 660, Chassis_Data.vx);
        float vy_slope = ApplySlope(slope_vy, TAR_LY * 660, Chassis_Data.vy);
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
        state_num = 1;
    } else if (Mode::Chassis::Universal()) {
        m_currentState = State::UniversalState;
        state_num = 2;
    } else if (Mode::Chassis::Follow()) {
        m_currentState = State::FollowState;
        state_num = 3;
    } else if (Mode::Chassis::Rotating()) {
        m_currentState = State::RotatingState;
        state_num = 4;
    } else if (Mode::Chassis::KeyBoard()) {
        m_currentState = State::KeyBoardState;
        state_num = 5;
    } else if (Mode::Chassis::Stop()) {
        m_currentState = State::StopState;
        state_num = 6;
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
    stringIk.StringInvKinematics(Chassis_Data.vx / 660.0f, Chassis_Data.vy / 660.0f, Chassis_Data.vw / 660.0f, 0.0f, 5.0f, 10.0f);

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

void Chassis_Task::PID_Updata()
{   
    // 舵向电机更新
    for (int i = 0; i < 4; i++)
    {
        // 舵向电机前馈更新
        feed_4005[i].UpData(Chassis_Data.FF_Zero_cross[i]);
        Chassis_Data.FF_Zero_cross[i] = Tools.Round_Error(feed_4005[i].cout, feed_4005[i].target_e, 360);

        pid_angle_String[i].GetPidPos(Kpid_4005_angle, Chassis_Data.Zero_cross[i], BSP::Motor::LK::Motor4005.getAngleDeg(i+1), 5000);
        
        // 舵向电机速度环更新
        pid_vel_String[i].GetPidPos(Kpid_4005_vel, pid_angle_String[i].pid.cout, BSP::Motor::LK::Motor4005.getVelocityRpm(i+1), 500);
        
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
    }

   // 功率控制部分
//    if (is_ude || Dir_Event.GetDir_String() == false)
//    {
//       PowerControl.String_PowerData.UpScaleMaxPow(pid_vel_String);
//       PowerControl.String_PowerData.UpCalcMaxTorque(Chassis_Data.final_4005_Out, pid_vel_String,
//                                                     toque_const_4005, rpm_to_rads_4005);                                          
//    }
//    if (is_ude || Dir_Event.GetDir_Wheel() == false)
//    {
//       PowerControl.Wheel_PowerData.UpScaleMaxPow(pid_vel_Wheel);
//       PowerControl.Wheel_PowerData.UpCalcMaxTorque(Chassis_Data.final_3508_Out, pid_vel_Wheel,
//                                                    toque_const_3508, rpm_to_rads_3508);
//    }
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
        for (int i = 0; i < 4; i++)
        {
            Chassis_Data.final_4005_Out[i] = pid_vel_String[i].GetCout();
            iqControl[i] = (int16_t)Chassis_Data.final_4005_Out[i];
        }
        BSP::Motor::LK::Motor4005.ctrl_Multi(iqControl);
    }
    if (Send_ms == 0)
    {
        BSP::Motor::Dji::Motor3508.sendCAN();
    }

    Send_ms ++;         
    Send_ms %= 2;  

//    Tools.vofaSend(BSP::Motor::Dji::Motor3508.getVelocityRpm(1), BSP::Motor::Dji::Motor3508.getVelocityRpm(2),
//                    BSP::Motor::Dji::Motor3508.getVelocityRads(3), BSP::Motor::Dji::Motor3508.getVelocityRads(4), 
//                    0, 0);
}   



