/*
#ifndef CTRL_STEP_FW_TB67H450_BASE_H
#define CTRL_STEP_FW_TB67H450_BASE_H

#include "driver_base.h"

class TB67H450Base : public DriverBase
{
public:
    explicit TB67H450Base()
    = default;

    void Init() override;

    void SetFocCurrentVector(uint32_t _directionInCount, int32_t _current_mA) override;

    void Sleep() override;

    void Brake() override;


protected:
    void SetTwoCoilsCurrent(uint16_t _currentA_3300mAIn12Bits, uint16_t _currentB_3300mAIn12Bits) override;


    /***** Port Specified Implements ****#1#
    virtual void InitGpio();

    virtual void InitPwm();

    virtual void DacOutputVoltage(uint16_t _voltageA_3300mVIn12bits, uint16_t _voltageB_3300mVIn12bits);

    virtual void SetInputA(bool _statusAp, bool _statusAm);

    virtual void SetInputB(bool _statusBp, bool _statusBm);
};

#endif
*/






#ifndef CTRL_STEP_FW_TB67H450_BASE_H
#define CTRL_STEP_FW_TB67H450_BASE_H

#include "driver_base.h"

/**
 * @brief TB67H450 步进电机驱动基类
 * @note  本类实现了通用算法层（FOC 电流矢量、Sleep、Brake 等），
 *        但底层的 GPIO / PWM / DAC 输出由具体平台类继承实现。
 *        例如：TB67H450_STM32 或 TB67H450_GD32。
 */
class TB67H450Base : public DriverBase
{
public:
    explicit TB67H450Base() = default;

    /**
     * @brief 初始化函数（调用底层硬件初始化）
     */
    void Init() override;

    /**
     * @brief 设置 FOC 电流矢量（方向 + 电流）
     * @param _directionInCount  电角度计数（0~1023）
     * @param _current_mA        目标电流值（单位 mA）
     */
    void SetFocCurrentVector(uint32_t _directionInCount, int32_t _current_mA) override;

    /**
     * @brief 电机进入休眠状态（关闭输出）
     */
    void Sleep() override;

    /**
     * @brief 电机制动（双线短接）
     */
    void Brake() override;

protected:
    /**
     * @brief 设置两相绕组的 DAC 输出电流
     * @param _currentA_3300mAIn12Bits 相A 电流12位值
     * @param _currentB_3300mAIn12Bits 相B 电流12位值
     */
    void SetTwoCoilsCurrent(uint16_t _currentA_3300mAIn12Bits,
                            uint16_t _currentB_3300mAIn12Bits) override;

    /***** 以下函数由具体平台实现 *****/

    /**
     * @brief 初始化GPIO引脚
     * @note  不同MCU平台的GPIO配置不同，需子类实现
     */
    virtual void InitGpio() = 0;

    /**
     * @brief 初始化PWM模块
     * @note  子类负责启动PWM通道（如HAL_TIM_PWM_Start）
     */
    virtual void InitPwm() = 0;

    /**
     * @brief 通过DAC输出两相电压
     * @param _voltageA_3300mVIn12bits 相A目标电压 (0~4095)
     * @param _voltageB_3300mVIn12bits 相B目标电压 (0~4095)
     */
    virtual void DacOutputVoltage(uint16_t _voltageA_3300mVIn12bits,
                                  uint16_t _voltageB_3300mVIn12bits) = 0;

    /**
     * @brief 设置相A输入引脚状态
     * @param _statusAp 高侧开关状态
     * @param _statusAm 低侧开关状态
     */
    virtual void SetInputA(bool _statusAp, bool _statusAm) = 0;

    /**
     * @brief 设置相B输入引脚状态
     * @param _statusBp 高侧开关状态
     * @param _statusBm 低侧开关状态
     */
    virtual void SetInputB(bool _statusBp, bool _statusBm) = 0;
};

#endif // CTRL_STEP_FW_TB67H450_BASE_H







