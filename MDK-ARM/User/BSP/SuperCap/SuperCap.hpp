#include "../Bsp_Can.hpp"
#include "../BSP/StaticTime.hpp"
#include "can.h"
#include <memory>
#include "../HAL/CAN/can_hal.hpp"
#include "../BSP/state_watch.hpp"
namespace BSP::SuperCap
{
class LH_Cap
{
  public:
    // 获取单例实例

    void ParseData(const CAN_RxHeaderTypeDef RxHeader, const uint8_t *pData)
    {
        const uint16_t received_id = HAL::CAN::ICanDevice::extract_id(RxHeader);
        if (received_id == 0x233)
        {
            std::memcpy(&feedback_, pData, sizeof(feedback));

            feedback_.In_Power = __builtin_bswap16(feedback_.In_Power);
            feedback_.Cap_Voltage = __builtin_bswap16(feedback_.Cap_Voltage);
            feedback_.Out_Power = __builtin_bswap16(feedback_.Out_Power);

            UpdateStatus();
            state_watch_.UpdateLastTime();
            state_watch_.UpdateTime();
            state_watch_.CheckStatus();
        }
    }

    void SetSendValue(uint16_t send_power)
    {
        send_data[0] = send_power >> 8;
        send_data[1] = send_power;
    }

    void UpdateStatus()
    {
        CapData_.In_Power = feedback_.In_Power / 100.0f;
        CapData_.Cap_Voltage = feedback_.Cap_Voltage / 100.0f;
        CapData_.Out_Power = feedback_.Out_Power / 100.0f;

        CapState_ = static_cast<State>(feedback_.State);
        CapSwitch_ = static_cast<Switch>(feedback_.Is_On);
		
		// dirTime.UpLastTime();
    }
    void Parse(const HAL::CAN::Frame& frame)
    {
        CAN_RxHeaderTypeDef rx_header;
        rx_header.StdId = frame.id;
        rx_header.ExtId = frame.id;
        rx_header.IDE = frame.is_extended_id ? CAN_ID_EXT : CAN_ID_STD;
        rx_header.RTR = frame.is_remote_frame ? CAN_RTR_REMOTE : CAN_RTR_DATA;
        rx_header.DLC = frame.dlc;
        
        ParseData(rx_header, frame.data);
    }
    enum class State : uint8_t
    {
        NORMAL,
        WARNING,
        ERROR
    };

    enum class Switch : uint8_t
    {
        ENABLE,
        DISABLE
    };

  private:
    struct alignas(uint64_t) feedback
    {
        int16_t In_Power;
        int16_t Cap_Voltage;
        int16_t Out_Power;

        uint8_t State;
        uint8_t Is_On;
    };

    struct data
    {
        float In_Power;
        float Cap_Voltage;
        float Out_Power;
    };

    feedback feedback_;
    data CapData_;
    State CapState_ = State::NORMAL;
    Switch CapSwitch_ = Switch::DISABLE;
    uint8_t send_data[8];
    uint32_t sendID = 0x666;

	//RM_StaticTime dirTime;
	bool Dir_Flag = false;
	
	// 添加状态监视器
    BSP::WATCH_STATE::StateWatch state_watch_{100}; // 100ms超时

  public:
    /**
     * @brief 获取底盘电压
     *
     * @return float
     */
    float getInPower()
    {
        return CapData_.In_Power;
    }

    /**
     * @brief 获取电容电压
     *
     * @return float
     */
    float getCapVoltage()
    {
        return CapData_.Cap_Voltage;
    }

    /**
     * @brief 获取输出功率
     *
     * @return float
     */
    float getOutPower()
    {
        return CapData_.Out_Power;
    }

    /**
     * @brief 获取电容状态
     *
     * @return State
     */
    State getCapState()
    {
        return CapState_;
    }

    /**
     * @brief 获取电容开关状态
     *
     * @return Switch
     */
    Switch getCapSwitch()
    {
        return CapSwitch_;
    }

    void sendCAN(CAN_HandleTypeDef *han, uint32_t pTxMailbox)
    {
        // 发送
        // auto& can_bus = HAL::CAN::get_can_bus_instance();
        // auto& can_device = can_bus.get_device(HAL::CAN::CanDeviceId::HAL_Can2);

        // HAL::CAN::Frame frame;
        // frame.id = sendID;
        // frame.dlc = 8;
        // frame.is_extended_id = false;
        // frame.is_remote_frame = false;
        // std::memcpy(frame.data, send_data, 8);

        // can_device.send(frame);
    }
	
	bool isScOnline()
	{
        return (state_watch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE);
	}
};

inline LH_Cap cap;
} // namespace BSP::SuperCap