# 底盘调试说明

这份文档基于当前代码现状重新整理。

目的不是泛泛介绍工程，而是回答下面几个实际问题：

1. 现在到底还有哪些地方是挖空的。
2. 哪些地方已经恢复，不需要再补。
3. 如果要把整个底盘从“能编译”调到“能稳定跑”，应该按什么顺序做。

## 1. 当前代码状态总览

### 已保留原始逻辑，不需要补

- 功率控制任务：`F:\H_SG\MDK-ARM\User\Task\PowerTask.cpp`
- 裁判系统 / UI 任务：`F:\H_SG\MDK-ARM\User\Task\RefeeTask.cpp`
- 板间通信主循环：`F:\H_SG\MDK-ARM\User\Task\CommunicationTask.cpp` 里的 `CommunicationTask()`
- 板间通信接收解析：`F:\H_SG\MDK-ARM\User\Task\CommunicationTask.cpp` 里的
  `ParseCANFrame()`、`ProcessReceivedData()`、`ShouldTransmit()`
- 板间通信超时判断：`F:\H_SG\MDK-ARM\User\Task\CommunicationTask.cpp` 里的 `PollLinkRecovery()`

### 仍然处于挖空状态，需要补

主要还剩三块：

- 底盘控制主链路：`Task\ChassisTask.cpp`
- 板间通信的“业务解析”和“发包”：`Task\CommunicationTask.cpp`
- 在线检测：`Task\EvenTask.cpp`

## 2. 当前仍然挖空的具体函数

## 2.1 底盘控制

文件：

- `F:\H_SG\MDK-ARM\User\Task\ChassisTask.cpp`

当前仍需要补的函数：

- `void Chassis_Task::Wheel_UpData()`
- `void Chassis_Task::Filtering()`
- `void Chassis_Task::PID_Updata()`
- `void Chassis_Task::CAN_Setting()`
- `void Chassis_Task::CAN_Send()`

其中状态如下：

- `Wheel_UpData()`：当前只把 `tar_speed` / `tar_angle` 清零，没有真正做逆运动学
- `Filtering()`：当前只把 `td_3508_speed[i]` 输入设成 `0.0f`
- `PID_Updata()`：函数前半段直接清 PID 并 `return`，后面的原始 PID 代码不会执行
- `CAN_Setting()`：函数前半段把最终输出清零并 `return`，后面的原始分配代码不会执行
- `CAN_Send()`：函数前半段只改了 `Send_ms` 然后 `return`，真正的电机发送代码不会执行

## 2.2 板间通信

文件：

- `F:\H_SG\MDK-ARM\User\Task\CommunicationTask.cpp`

当前仍需要重点关注的函数：

- `void Gimbal_to_Chassis::ProcessReceivedData()`
- `void Gimbal_to_Chassis::Transmit()`

其中状态如下：

- `ProcessReceivedData()`：当前 CAN 解包主流程还在，但这里决定了“收到云台数据后，底盘到底如何更新业务状态、是否通知上层模块”，所以它才是板间通信真正要先看懂的地方
- `Transmit()`：函数最前面直接 `return`，后面的业务打包和 CAN2 发包代码不会执行

注意：

- `CommunicationTask()` 主循环本身已经恢复
- `ParseCANFrame()` 和 `ProcessReceivedData()` 才是当前板间通信业务链路的核心
- `PollLinkRecovery()` / `RecoverCanReceiver()` 更偏异常恢复，不是你现在第一优先级

## 2.3 在线检测

文件：

- `F:\H_SG\MDK-ARM\User\Task\EvenTask.cpp`

当前仍需要补的函数：

- `bool Dir::Dir_String()`
- `bool Dir::Dir_Wheel()`
- `bool Dir::Dir_MeterPower()`
- `bool Dir::Dir_Communication()`
- `bool Dir::Dir_SuperCap()`

其中状态如下：

- 这些函数前半段都先写死成了 `true`
- 后面的原始检测代码还在，但由于前面已经 `return true`，所以不会执行

## 3. 当前不建议再改的部分

下面这些现在已经是有效逻辑，调试时不要误以为它们还需要补：

### 功率控制

文件：

- `F:\H_SG\MDK-ARM\User\Task\PowerTask.cpp`

说明：

- `RLSTask()` 已恢复
- 动态功率上限、能量模式、超级电容策略、VOFA 调试输出都还在

结论：

- 不要再把功率控制当成挖空点
- 在底盘跑起来之前，只需要确认它没有被上层输出链路卡死即可

### 裁判系统 / UI

文件：

- `F:\H_SG\MDK-ARM\User\Task\RefeeTask.cpp`

说明：

- 静态 UI 初始化在
  `UI::Static::UI_static.Init()`
- 动态 UI 刷新在
  `UI::Dynamic::UI_dynamic.darw_UI()`
- 发送队列在
  `UI::UI_send_queue.send_wz()` 和 `UI::UI_send_queue.send()`

结论：

- 不要把 UI 当成当前主问题
- 如果 UI 不正常，先检查裁判系统链路，不要先去改 `RefeeTask.cpp`

## 4. 每个挖空点应该去看哪里

## 4.1 `Chassis_Task::Wheel_UpData()`

要看：

- `F:\H_SG\MDK-ARM\User\Algorithm\ChassisCalculation\StringWheel.hpp`
- `F:\H_SG\MDK-ARM\User\BSP\Motor\Lk\Lk_motor.hpp`

优先用这些接口：

- `Alg::CalculationBase::String_IK::Set_current_steer_angles(angle, index)`
- `Alg::CalculationBase::String_IK::StringInvKinematics(vx, vy, vw, phase, speed_gain, rotate_gain)`
- `Alg::CalculationBase::String_IK::GetMotor_wheel(index)`
- `Alg::CalculationBase::String_IK::GetMotor_direction(index)`
- `BSP::Motor::LK::Motor4005.getAngleRad(i + 1)`

你要完成的事：

- 根据 `Chassis_Data.vx / vy / vw`
- 结合 4 个舵向电机当前角度
- 算出 4 个轮子的目标速度和目标角度
- 写回 `Chassis_Data.tar_speed[i]`、`Chassis_Data.tar_angle[i]`

## 4.2 `Chassis_Task::Filtering()`

要看：

- `F:\H_SG\MDK-ARM\User\BSP\Motor\Dji\DjiMotor.hpp`
- `F:\H_SG\MDK-ARM\User\Algorithm\PID.hpp`

优先用这些接口：

- `BSP::Motor::Dji::Motor3508.getVelocityRads(i + 1)`
- `td_3508_speed[i].Calc(feedback_speed)`

你要完成的事：

- 读取驱动轮实际速度
- 更新 `td_3508_speed[i]`
- 给后面的速度环提供稳定反馈

## 4.3 `Chassis_Task::PID_Updata()`

要看：

- `F:\H_SG\MDK-ARM\User\Algorithm\PID.hpp`
- `F:\H_SG\MDK-ARM\User\BSP\Motor\Lk\Lk_motor.hpp`
- `F:\H_SG\MDK-ARM\User\BSP\Motor\Dji\DjiMotor.hpp`

优先用这些接口：

- `PID::GetPidPos(...)`
- `PID::GetCout()`
- `PID::clearPID()`
- `BSP::Motor::LK::Motor4005.getAngleDeg(i + 1)`
- `BSP::Motor::LK::Motor4005.getVelocityRpm(i + 1)`
- `td_3508_speed[i].x1`

你要完成的事：

- 舵向双环：角度环 + 速度环
- 驱动轮速度环
- 最终把 PID 输出保存在对应 PID 对象里

## 4.4 `Chassis_Task::CAN_Setting()`

要看：

- `F:\H_SG\MDK-ARM\User\Task\PowerTask.hpp`
- `F:\H_SG\MDK-ARM\User\BSP\Motor\Dji\DjiMotor.hpp`

优先用这些接口：

- `pid_vel_Wheel[i].GetCout()`
- `pid_vel_String[i].GetCout()`
- `PowerControl.String_PowerData.UpScaleMaxPow(...)`
- `PowerControl.String_PowerData.UpCalcMaxTorque(...)`
- `PowerControl.Wheel_PowerData.UpScaleMaxPow(...)`
- `PowerControl.Wheel_PowerData.UpCalcMaxTorque(...)`
- `BSP::Motor::Dji::Motor3508.setCAN(value, i + 1)`

你要完成的事：

- 从 PID 里取原始输出
- 做二次处理和功率限制
- 生成 `final_3508_Out` / `final_4005_Out`
- 同时把 3508 的 CAN 缓冲写好

## 4.5 `Chassis_Task::CAN_Send()`

要看：

- `F:\H_SG\MDK-ARM\User\BSP\Motor\Lk\Lk_motor.hpp`
- `F:\H_SG\MDK-ARM\User\BSP\Motor\Dji\DjiMotor.hpp`
- `F:\H_SG\MDK-ARM\User\BSP\SuperCap\SuperCap.hpp`

优先用这些接口：

- `BSP::Motor::LK::Motor4005.ctrl_Multi(iq_control)`
- `BSP::Motor::Dji::Motor3508.sendCAN()`
- `BSP::SuperCap::cap.sendCAN()`

你要完成的事：

- 把舵向电机控制量打包发送
- 把驱动轮 CAN 帧发出去
- 保留原本的交替发送节奏

## 4.6 `Gimbal_to_Chassis::ProcessReceivedData()`

要看：

- `F:\H_SG\MDK-ARM\User\Task\CommunicationTask.hpp`
- `F:\H_SG\MDK-ARM\User\Task\CommunicationTask.cpp`

优先关注这些内容：

- `direction`
- `chassis_mode`
- `ui_list`
- `UI::Refresh::NotifyUiDataUpdated()`
- `state_watch_.UpdateLastTime()`
- `state_watch_.UpdateTime()`
- `state_watch_.CheckStatus()`

你要先搞清楚的事：

- 云台发来的两帧 CAN 数据是怎么拼起来的
- `ProcessReceivedData()` 解包后，哪些量会进入底盘控制链路
- 哪些量会影响 UI、模式切换、功率策略

说明：

- 如果板间通信“能收到但行为不对”，优先查这里
- 如果板间通信“完全收不到或掉线后不恢复”，再去查 `RecoverCanReceiver()`

## 4.7 `Gimbal_to_Chassis::Transmit()`

要看：

- `F:\H_SG\MDK-ARM\User\Task\CommunicationTask.hpp`
- `F:\H_SG\MDK-ARM\User\HAL\CAN\can_hal.hpp`

当前代码后半段原始参考已经保留，主要数据源包括：

- `ext_power_heat_data_0x0202.shooter_id1_17mm_cooling_heat`
- `ext_power_heat_data_0x0201.shooter_barrel_heat_limit`
- `ext_power_heat_data_0x0201.shooter_barrel_cooling_value`
- `ext_shoot_data_0x0207.initial_speed`

你要完成的事：

- 取消前面的提前 `return`
- 根据现有 `booster` 协议打包
- 通过 CAN2 发给云台

可以一起关注的配套调用：

- `Gimbal_to_Chassis_Data.ShouldTransmit()`
- `Gimbal_to_Chassis_Data.NotifyCanError(error)`
- `state_watch_.UpdateLastTime()`
- `state_watch_.UpdateTime()`
- `state_watch_.CheckStatus()`

## 4.8 `EvenTask.cpp` 里的在线检测

要看：

- `F:\H_SG\MDK-ARM\User\BSP\Motor\Lk\Lk_motor.hpp`
- `F:\H_SG\MDK-ARM\User\BSP\Motor\Dji\DjiMotor.hpp`
- `F:\H_SG\MDK-ARM\User\BSP\Power\PM01.hpp`
- `F:\H_SG\MDK-ARM\User\BSP\SuperCap\SuperCap.hpp`
- `F:\H_SG\MDK-ARM\User\BSP\Common\StateWatch\buzzer_manager.hpp`

优先用这些接口：

- `BSP::Motor::LK::Motor4005.isMotorOnline(i + 1)`
- `BSP::Motor::Dji::Motor3508.isMotorOnline(i + 1)`
- `BSP::Power::pm01.isPmOnline()`
- `BSP::SuperCap::cap.isScOnline()`
- `Gimbal_to_Chassis_Data.isConnectOnline()`
- `BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestMotorRing(i + 1)`
- `BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestCommunicationRing()`

补在线检测时建议顺手补的逻辑：

- 舵向电机掉线时，调用
  `BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestMotorRing(i + 1)`
- 驱动轮电机掉线时，调用
  `BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestMotorRing(i + 1)`
- 板间通信掉线时，调用
  `BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestCommunicationRing()`
- 每轮检测结束后，调用
  `BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update()`

## 5. 现在调试整车，推荐顺序

你当前的目标是：

1. 先把底盘本体调通
2. 底盘能稳定跑之后，再加错误报警
3. 最后再加板间通信

这个顺序是合理的。

因为现在最核心的问题不是“系统功能全不全”，而是“底盘这条主控制链有没有真正闭合”。

### 第一步：先只调底盘本体

先只关注：

- `Wheel_UpData()`
- `Filtering()`
- `PID_Updata()`
- `CAN_Setting()`
- `CAN_Send()`

这一阶段先不要把精力放在：

- 蜂鸣器报警
- 板间通信
- 通信掉线恢复

原因：

- 如果底盘主链路没通，先接报警和通信只会增加干扰项
- 你现在最需要先确认的是“给目标值以后，车能不能按预期动起来”

### 第二步：先补底盘逆运动学

对应：

- `Wheel_UpData()`

目标：

- 先让 `tar_speed`、`tar_angle` 变成合理值
- 还不需要立刻闭环控制

验证方式：

- 给定固定 `vx / vy / vw`
- 看四个轮子的目标角度是否符合预期
- 看舵向是否会选择最近转角

### 第三步：补反馈滤波

对应：

- `Filtering()`

目标：

- 让速度环拿到真实反馈，而不是全 0

### 第四步：补 PID

对应：

- `PID_Updata()`

建议顺序：

1. 先只调舵向双环
2. 再调驱动轮速度环
3. 最后再一起联动

不要一上来就四轮一起大范围跑。

### 第五步：补输出分配

对应：

- `CAN_Setting()`

目标：

- 把 PID 输出变成实际可下发的控制量
- 接回功率限制

注意：

- 功率控制模块本身已经是好的
- 现在真正断掉的是“PID 输出有没有被送进功率分配和电机发送”

### 第六步：补 CAN 下发

对应：

- `CAN_Send()`

目标：

- 真正把 4005、3508、超级电容控制量发出去

### 第七步：整车联调底盘本体

到这一步，目标是“底盘本体稳定跑起来”。

建议顺序：

1. 先只测舵向
2. 再只测驱动轮
3. 再测纯平移
4. 再测纯旋转
5. 最后测平移加旋转

如果这一步完成，说明底盘本体已经基本调通。

### 第八步：再补错误报警

这一步再去处理：

- `Dir_String()`
- `Dir_Wheel()`
- `Dir_MeterPower()`
- `Dir_Communication()`
- `Dir_SuperCap()`

原因：

- 当底盘已经能正常跑时，再加报警逻辑，更容易区分“控制问题”和“离线问题”
- 这时候补蜂鸣器提示也更有意义

建议顺手补的调用：

- `BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestMotorRing(i + 1)`
- `BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestCommunicationRing()`
- `BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update()`

### 第九步：最后补板间通信

最后再处理：

- `ProcessReceivedData()`
- `Transmit()`
- 有需要时再看 `RecoverCanReceiver()`

原因：

- 只有在底盘本体已经稳定时，板间通信问题才容易单独定位
- 否则你会分不清到底是“云台传错了”，还是“底盘本体控制就没通”

这个阶段建议按下面顺序：

1. 先看懂 `ProcessReceivedData()` 解包后哪些变量会进入底盘控制链
2. 再补 `Transmit()`，确认底盘能把状态回传给云台
3. 最后再处理异常恢复，例如 `RecoverCanReceiver()`

## 6. 现阶段最关键的判断原则

当前这份代码里，最容易误判的问题有两个：

### 误判一：以为功率控制有问题

不是。

现在功率控制 `RLSTask()` 已恢复。
如果车完全不动，优先检查的是：

- `PID_Updata()` 是否还在提前 `return`
- `CAN_Setting()` 是否还在提前 `return`
- `CAN_Send()` 是否还在提前 `return`

### 误判二：以为通信已经完全恢复

不是。

现在：

- 收包在
- 解析在
- 超时判断在

但是：

- `Transmit()` 还没真正发包
- `ProcessReceivedData()` 虽然在运行，但它仍然是你理解和核对板间通信业务链路的第一入口

所以当前通信问题应优先按“解包业务链路”来查，再按“异常恢复链路”来查。

补充一点：

- 如果板间通信掉线，除了恢复 CAN，本工程还建议给出显式提示
- 最直接的调用就是
  `BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestCommunicationRing()`
- 这样现场调试时，不看串口也能第一时间知道是通信链路出问题

## 7. 建议的实际补代码顺序

如果按照你现在的计划推进，建议严格按这个顺序：

1. `Task\ChassisTask.cpp` 的 `Wheel_UpData()`
2. `Task\ChassisTask.cpp` 的 `Filtering()`
3. `Task\ChassisTask.cpp` 的 `PID_Updata()`
4. `Task\ChassisTask.cpp` 的 `CAN_Setting()`
5. `Task\ChassisTask.cpp` 的 `CAN_Send()`
6. 底盘联调，确认车能稳定跑
7. `Task\EvenTask.cpp`
8. `Task\CommunicationTask.cpp` 的 `ProcessReceivedData()`
9. `Task\CommunicationTask.cpp` 的 `Transmit()`
10. 有需要时再看 `Task\CommunicationTask.cpp` 的 `RecoverCanReceiver()`

不要跳步。

现在真正影响你“先把底盘跑起来”的核心点，是 `ChassisTask.cpp` 里这些提前 `return` 和占位逻辑。

## 8. 调试时每一步都要确认什么

每补完一块，都至少检查下面四件事：

1. 输入是不是已经真实有效
2. 输出是不是写回了正确变量
3. 有没有因为提前 `return` 导致后面逻辑根本没跑
4. 单位是不是统一了

特别注意单位：

- 舵向角有弧度和角度两套
- 3508 和 4005 的控制量范围不同
- 逆运动学输出、PID 输入、CAN 输出不要混单位

## 9. 最后一句

现在这套代码不是“大面积都没写”，而是“主框架在，关键执行链路有几处被人为截断了”。

所以正确做法不是重写，而是按链路一点点接回去：

- 底盘逆解
- 反馈
- PID
- 输出分配
- CAN 下发
- 先让底盘跑起来
- 再加错误报警
- 最后接板间通信

顺序对了，整车会快很多。
