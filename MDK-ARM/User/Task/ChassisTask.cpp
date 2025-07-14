#include "ChassisTask.hpp"
#include "../APP/Referee/RM_RefereeSystem.h"
#include "../Task/CommunicationTask.hpp"
#include "HAL.hpp"
#include "State.hpp"
#include "Variable.hpp"
#include "cmsis_os2.h"

#include "../APP/Remote/KeyBroad.hpp"
#include "../APP/Remote/Mode.hpp"
#include "../APP/UI/Static/darw_static.hpp"
#include "../APP/UI/UI_Queue.hpp"
#include "../BSP/Dbus.hpp"
#include "../BSP/Power/PM01.hpp"
TaskManager taskManager;


void ChassisTask(void *argument)
{
    osDelay(500);

    taskManager.addTask<Chassis_Task>();

    for (;;)
    {
        taskManager.updateAll();
        osDelay(2);
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
        auto cos_theta = HAL::cosf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);
        auto sin_theta = HAL::sinf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);

        // 设置斜坡目标值
        slope_vx.Set_Target(TAR_LX * 660);
        slope_vy.Set_Target(TAR_LY * 660);

        // 更新真实值（从当前速度获取）
        slope_vx.Set_Now_Real(Chassis_Data.vx);
        slope_vy.Set_Now_Real(Chassis_Data.vy);

        // 计算斜坡输出
        slope_vx.TIM_Calculate_PeriodElapsedCallback();
        slope_vy.TIM_Calculate_PeriodElapsedCallback();

        // 获取斜坡输出
        Chassis_Data.vx = slope_vx.Get_Out();
        Chassis_Data.vy = slope_vy.Get_Out();

        Chassis_Data.vx = (Chassis_Data.vx * cos_theta - Chassis_Data.vy * sin_theta);
        Chassis_Data.vy = (Chassis_Data.vx * sin_theta + Chassis_Data.vy * cos_theta);
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
        auto cos_theta = HAL::cosf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);
        auto sin_theta = HAL::sinf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);

        slope_vx.Set_Target(TAR_LX * 660);
        slope_vy.Set_Target(TAR_LY * 660);

        pid_vw.GetPidPos(Kpid_vw, 0, Gimbal_to_Chassis_Data.getEncoderAngleErr(), 10000);
        slope_vw.Set_Target(pid_vw.GetCout());
        // 更新真实值（从当前速度获取）
        slope_vx.Set_Now_Real(Chassis_Data.vx);
        slope_vy.Set_Now_Real(Chassis_Data.vy);
        slope_vw.Set_Now_Real(Chassis_Data.vw);

        // 计算斜坡输出
        slope_vx.TIM_Calculate_PeriodElapsedCallback();
        slope_vy.TIM_Calculate_PeriodElapsedCallback(); 
        slope_vw.TIM_Calculate_PeriodElapsedCallback();
        // 获取斜坡输出
        Chassis_Data.vx = slope_vx.Get_Out();
        Chassis_Data.vy = slope_vy.Get_Out();
        Chassis_Data.vw = slope_vw.Get_Out();
        
        Chassis_Data.vx = (Chassis_Data.vx * cos_theta - Chassis_Data.vx * sin_theta);
        Chassis_Data.vy = (Chassis_Data.vy * sin_theta + Chassis_Data.vy * cos_theta);
        Chassis_Data.vw = (Chassis_Data.vw);

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
        auto cos_theta = HAL::cosf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);
        auto sin_theta = HAL::sinf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle);

        tar_vx.Calc(TAR_LX * 660);
        tar_vy.Calc(TAR_LY * 660);

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

        Chassis_Data.vx = (tar_vx.x1 * cos_theta - tar_vy.x1 * sin_theta);
        Chassis_Data.vy = (tar_vx.x1 * sin_theta + tar_vy.x1 * cos_theta);
        Chassis_Data.vw = (tar_vw.x1);

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
<<<<<<< Updated upstream
float ROTATION_BIAS = 0.1f;
=======

>>>>>>> Stashed changes
class Chassis_Task::RotatingHandler : public StateHandler
{
  public:
    Chassis_Task &m_task;

    explicit RotatingHandler(Chassis_Task &task) : m_task(task)
    {
    }

    void RotatingTarget()
    {
<<<<<<< Updated upstream
        auto cos_theta = HAL::cosf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle + ROTATION_BIAS * Chassis_Data.vw);
        auto sin_theta = HAL::sinf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle + ROTATION_BIAS * Chassis_Data.vw);
=======
        auto cos_theta = HAL::cosf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle + k * Chassis_Data.vw);
        auto sin_theta = HAL::sinf(-Gimbal_to_Chassis_Data.getEncoderAngleErr() + tar_vw_angle + k * Chassis_Data.vw);
>>>>>>> Stashed changes

        tar_vx.Calc(TAR_LX * 660);
        tar_vy.Calc(TAR_LY * 660);
        tar_vw.Calc(TAR_VW * 660);

        //        pid_vw.GetPidPos(Kpid_vw, 0, Gimbal_to_Chassis_Data.getEncoderAngleErr(), 10000);
        //        tar_vw.Calc(pid_vw.GetCout());

        Chassis_Data.vx = (tar_vx.x1 * cos_theta - tar_vy.x1 * sin_theta);
        Chassis_Data.vy = (tar_vx.x1 * sin_theta + tar_vy.x1 * cos_theta);
        Chassis_Data.vw = (tar_vw.x1);

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

    void RotingTarget()
    {
        tar_vx.Calc(TAR_LX * 660);
        tar_vy.Calc(TAR_LY * 660);
        td_FF_Tar.Calc(TAR_LX * 660);

        if (CONTROL_SIG == 0)
            tar_vw.Calc(660);
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
        m_task.applyRearBrake(0.5);    
        // // 设置目标为0并增大减量值
        // slope_vx.Set_Target(0);
        // slope_vy.Set_Target(0);
        // slope_vw.Set_Target(0);
        
        // // 获取斜坡输出
        // Chassis_Data.vx = slope_vx.Get_Out();
        // Chassis_Data.vy = slope_vy.Get_Out();
        // Chassis_Data.vw = slope_vw.Get_Out();

        // // 更新斜坡输出
        // slope_vx.TIM_Calculate_PeriodElapsedCallback();
        // slope_vy.TIM_Calculate_PeriodElapsedCallback();
        // slope_vw.TIM_Calculate_PeriodElapsedCallback();
        
        // PID_Updata();

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
          Class_Slope(2, 3, Slope_First_REAL)}
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

    if (Mode::Chassis::Universal())
    {
        m_currentState = State::UniversalState;
    }
    if (Mode::Chassis::Follow())
    {
        m_currentState = State::FollowState;
    }
    if (Mode::Chassis::Rotating())
    {
        m_currentState = State::RotatingState;
    }
    if (Mode::Chassis::KeyBoard())
    {
        m_currentState = State::KeyBoardState;
    }
    if (Mode::Chassis::Stop())
    {
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
    // 对轮子进行运动学变换
    Wheel.WheelType.UpDate(Chassis_Data.vx, Chassis_Data.vy, Chassis_Data.vw, 6000);

    // 储存最小角判断的速度
    for (int i = 0; i < 4; i++)
    {
        Chassis_Data.tar_speed[i] = Wheel.WheelType.speed[i];
    }

    // 储存最小角判断的角度
    for (int i = 0; i < 4; i++)
    {
        Chassis_Data.tar_angle[i] = Wheel.WheelType.angle[i];
    }

    // 进行最小角判断
    Chassis_Data.getMinPos[0] =
        Tools.MinPosHelm(Chassis_Data.tar_angle[0] + Chassis_angle_Init_0x205,
                         Motor6020.GetEquipData(L_Forward_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[0], 16384, 8192);
    Chassis_Data.getMinPos[1] =
        Tools.MinPosHelm(Chassis_Data.tar_angle[1] + Chassis_angle_Init_0x206,
                         Motor6020.GetEquipData(L_Back_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[1], 16384, 8192);
    Chassis_Data.getMinPos[2] =
        Tools.MinPosHelm(Chassis_Data.tar_angle[2] + Chassis_angle_Init_0x207,
                         Motor6020.GetEquipData(R_Back_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[2], 16384, 8192);
    Chassis_Data.getMinPos[3] =
        Tools.MinPosHelm(Chassis_Data.tar_angle[3] + Chassis_angle_Init_0x208,
                         Motor6020.GetEquipData(R_Forward_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[3], 16384, 8192);

    if (is_sin == true)
    {
        sin_t = 4096 + HAL::sinf(2 * 3.1415926 * ms * 0.001 * hz) * 4000;
        ude_tar = 4096 + HAL::sinf(2 * 3.1415926 * ms * 0.001 * hz) * 4000;
        ms++;
    }
    else
        sin_t = pos;

    // 过零处理         //发现直接用for会使电机疯
    // for(int i = 0; i < 4; i++)
    // {
    //    Chassis_Data.Zero_cross[i] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[i],
    //    Motor6020.GetAngleFeedback(i), 8192);
    // }
    Chassis_Data.Zero_cross[0] = Tools.Zero_crossing_processing(
        Chassis_Data.getMinPos[0], Motor6020.GetEquipData(L_Forward_6020_ID, Dji_Angle), 8192);
    Chassis_Data.Zero_cross[1] = Tools.Zero_crossing_processing(
        Chassis_Data.getMinPos[1], Motor6020.GetEquipData(L_Back_6020_ID, Dji_Angle), 8192);
    Chassis_Data.Zero_cross[2] = Tools.Zero_crossing_processing(
        Chassis_Data.getMinPos[2], Motor6020.GetEquipData(R_Back_6020_ID, Dji_Angle), 8192);
    Chassis_Data.Zero_cross[3] = Tools.Zero_crossing_processing(
        Chassis_Data.getMinPos[3], Motor6020.GetEquipData(R_Forward_6020_ID, Dji_Angle), 8192);
}
// 滤波器更新
void Chassis_Task::Filtering()
{
    // 电机一般速度反馈噪声大
    for (int i = 0; i < 4; i++)
    {
        td_3508_speed[i].Calc(Motor3508.GetRPMFeedback(i));
    }
}

void Chassis_Task::PID_Updata()
{
    // 舵向电机更新
    for (int i = 0; i < 4; i++)
    {
        // 舵向电机前馈更新
        feed_6020[i].UpData(Chassis_Data.Zero_cross[i]);
        Chassis_Data.FF_Zero_cross[i] = Tools.Round_Error(feed_6020[i].cout, feed_6020[i].target_e, 8191);

        // 舵向电机角度环更新
        pid_angle_String[i].GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[i], Motor6020.GetAngleFeedback(i),
                                      16384.0f);
        // 舵向电机速度环更新
        pid_vel_String[i].GetPidPos(Kpid_6020_vel, pid_angle_String[i].pid.cout, Motor6020.GetRPMFeedback(i), 16384);
    }

    // 轮毂电机速度环更新
    for (int i = 0; i < 4; i++)
    {
        pid_vel_Wheel[i].GetPidPos(Kpid_3508_vel, Chassis_Data.tar_speed[i], td_3508_speed[i].x1, 16384.0f);
    }
}

bool is_ude;
void Chassis_Task::CAN_Setting()
{
    for (int i = 0; i < 4; i++)
    {
        Chassis_Data.final_6020_Out[i] = pid_vel_String[i].GetCout();
    }

    // 如果，没有超功率就沿用pid输出，如果超功率就进入功率控制部分的判断
    for (int i = 0; i < 4; i++)
    {
        Chassis_Data.final_3508_Out[i] = pid_vel_Wheel[i].GetCout();
    }

//    // 功率控制部分
//    if (Dir_Event.GetDir_String() == false)
//    {
//        PowerControl.String_PowerData.UpScaleMaxPow(pid_angle_String, Motor6020);
//        PowerControl.String_PowerData.UpCalcMaxTorque(Chassis_Data.final_6020_Out, Motor6020, pid_vel_String,
//                                                      toque_const_6020, rpm_to_rads_6020);
//    }

//    if (Dir_Event.GetDir_Wheel() == false)
//    {
//        PowerControl.Wheel_PowerData.UpScaleMaxPow(pid_vel_Wheel, Motor3508);
//        PowerControl.Wheel_PowerData.UpCalcMaxTorque(Chassis_Data.final_3508_Out, Motor3508, pid_vel_Wheel,
//                                                     toque_const_3508, rpm_to_rads_3508);
//    }
    Motor6020.setMSD(&msd_6020, Chassis_Data.final_6020_Out[0], Get_MOTOR_SET_ID_6020(0x205));
    Motor6020.setMSD(&msd_6020, Chassis_Data.final_6020_Out[1], Get_MOTOR_SET_ID_6020(0x206));
    Motor6020.setMSD(&msd_6020, Chassis_Data.final_6020_Out[2], Get_MOTOR_SET_ID_6020(0x207));
    Motor6020.setMSD(&msd_6020, Chassis_Data.final_6020_Out[3], Get_MOTOR_SET_ID_6020(0x208));

    //    if (is_ude == true)
    //        Motor6020.setMSD(&msd_6020, ude_vel_demo.GetCout(), Get_MOTOR_SET_ID_6020(0x206));
    //    else
    //        Motor6020.setMSD(&msd_6020, ude_vel_demo.GetCout(), Get_MOTOR_SET_ID_6020(0x206));

    Motor3508.setMSD(&msd_3508_2006, Chassis_Data.final_3508_Out[0], Get_MOTOR_SET_ID_3508(0x201));
    Motor3508.setMSD(&msd_3508_2006, Chassis_Data.final_3508_Out[1], Get_MOTOR_SET_ID_3508(0x202));
    Motor3508.setMSD(&msd_3508_2006, Chassis_Data.final_3508_Out[2], Get_MOTOR_SET_ID_3508(0x203));
    Motor3508.setMSD(&msd_3508_2006, Chassis_Data.final_3508_Out[3], Get_MOTOR_SET_ID_3508(0x204));
}

void Chassis_Task::CAN_Send()
{
    // 发送数据
    if (Send_ms == 0)
    {
        BSP::Power::pm01.PM01SendFun();
        Motor3508.Send_CAN_MAILBOX1(&msd_3508_2006, SEND_MOTOR_ID_3508);
    }
    else if (Send_ms == 1)
    {
        Motor6020.Send_CAN_MAILBOX0(&msd_6020, SEND_MOTOR_CurrentID_6020);
    }

    Send_ms++;
    Send_ms %= 2;

    // Tools.vofaSend(BSP::Power::pm01.cin_power,
    //                PowerControl.String_PowerData.EstimatedPower,
    //                PowerControl.Wheel_PowerData.EstimatedPower,
    //                0,
    //                0,
    //                PowerControl.String_PowerData.pMaxPower[3]);
}
void  Chassis_Task::getRearWheels(int rearIndices[4])
{
    float chassis_dir[2] = {Chassis_Data.vx, Chassis_Data.vy};
    float len = sqrtf(chassis_dir[0] * chassis_dir[0] + chassis_dir[1] * chassis_dir[1]);

    int count = 0;

    if (len < 1e-6f)
    {
        // 静止时默认后轮为索引 1 和 2
        rearIndices[0] = 1;
        rearIndices[1] = 2;

    }

    // 归一化底盘方向
    chassis_dir[0] /= len;
    chassis_dir[1] /= len;

    float dotProducts[4];  // 每个轮子的方向与底盘方向的点积
    for (int i = 0; i < 4; ++i)
    {
        float angle = Chassis_Data.tar_angle[i];
        float speed = Chassis_Data.tar_speed[i];

        float wheel_dir[2] = {speed * cosf(angle), speed * sinf(angle)};
        float dot = wheel_dir[0] * chassis_dir[0] + wheel_dir[1] * chassis_dir[1];
        dotProducts[i] = dot;
    }

    // 找出最小的两个点积值（即反向最严重的两个轮子）
    int indices[4] = {0, 1, 2, 3};

    // 冒泡排序找最小两个
    for (int i = 0; i < 4; ++i)
    {
        for (int j = i + 1; j < 4; ++j)
        {
            if (dotProducts[i] > dotProducts[j])
            {
                std::swap(dotProducts[i], dotProducts[j]);
                std::swap(indices[i], indices[j]);
            }
        }
    }

    // 最小的两个即为后轮
    rearIndices[0] = indices[0];
    rearIndices[1] = indices[1];

}
void Chassis_Task::applyRearBrake(float brakeFactor)
{
    int rearIndices[2];
    int frontIndices[2];
    getRearWheels(rearIndices);

    // 找出前轮索引
    int idx = 0;
    for (int i = 0; i < 4; ++i)
    {
        bool isRear = (i == rearIndices[0] || i == rearIndices[1]);
        if (!isRear)
            frontIndices[idx++] = i;
    }

    // 后轮：直接置零（抱死）
    for (int i = 0; i < 2; ++i)
    {
        int idx = rearIndices[i];
        Chassis_Data.tar_speed[idx] = 0.0f;

        pid_vel_String[idx].clearPID();
        pid_vel_Wheel[idx].clearPID();
    }

    // 前轮：按比例衰减，模拟滑行效果
    for (int i = 0; i < 2; ++i)
    {
        int idx = frontIndices[i];
        float target = Chassis_Data.tar_speed[idx] * (1.0f - brakeFactor);

        // 设置斜坡目标
        slope_speed[idx].Set_Target(target);
        slope_speed[idx].Set_Now_Real(Chassis_Data.tar_speed[idx]);

        pid_vel_String[idx].clearPID();
        pid_vel_Wheel[idx].clearPID();
    }
}
