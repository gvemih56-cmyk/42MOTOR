#ifndef CTRL_STEP_FW_TB67H450_STM32_H
#define CTRL_STEP_FW_TB67H450_STM32_H

#include "Driver/tb67h450_base.h"   // 引入驱动芯片通用基类

/**
 * @brief TB67H450 步进电机驱动类（STM32 平台实现）
 * @note  TB67H450 是东芝推出的单通道步进电机驱动芯片，
 *        支持双极性控制，可驱动一相或两相步进电机。
 *
 *        本类继承自 TB67H450Base，负责完成芯片与STM32硬件层的对接。
 *        它通过 GPIO 控制相位方向，通过 PWM 或 DAC 控制电流幅值。
 */
class TB67H450 : public TB67H450Base
{
public:
    /**
     * @brief 构造函数
     * @note  调用基类构造函数完成基础初始化
     */
    explicit TB67H450() : TB67H450Base()
    {}

private:
    /*--------------------------------------------------------------
     * 底层硬件接口函数（重写基类虚函数）
     *--------------------------------------------------------------*/

    /**
     * @brief 初始化驱动芯片相关 GPIO 引脚
     * @note  包括相位控制引脚（AP/AM/BP/BM）、使能/休眠/故障检测引脚等。
     * @example
     *        AP → PA0
     *        AM → PA1
     *        BP → PB0
     *        BM → PB1
     *        ENABLE → PC13
     */
    void InitGpio() override;

    /**
     * @brief 初始化 PWM 输出通道
     * @note  用于控制电机绕组电流，通过占空比调整电流大小。
     *        一般使用定时器 PWM 模式输出。
     */
    void InitPwm() override;

    /**
     * @brief 输出模拟电压到 DAC 通道，用于控制相电流幅值
     * @param _voltageA_3300mVIn12bits  通道A目标电压值（单位：12bit）
     * @param _voltageB_3300mVIn12bits  通道B目标电压值（单位：12bit）
     * @note  TB67H450 通过模拟电压控制电流限制；
     *        例如 3.3V 参考下，12bit精度 = 0~4095。
     */
    void DacOutputVoltage(uint16_t _voltageA_3300mVIn12bits,
                          uint16_t _voltageB_3300mVIn12bits) override;

    /**
     * @brief 控制相A输入引脚状态（AP/AM）
     * @param _statusAp AP引脚电平（true=高电平）
     * @param _statusAm AM引脚电平（true=高电平）
     * @note  通过控制 AP / AM 引脚组合来决定电机A相的通电方向。
     */
    void SetInputA(bool _statusAp, bool _statusAm) override;

    /**
     * @brief 控制相B输入引脚状态（BP/BM）
     * @param _statusBp BP引脚电平（true=高电平）
     * @param _statusBm BM引脚电平（true=高电平）
     * @note  同上，用于控制电机B相。
     */
    void SetInputB(bool _statusBp, bool _statusBm) override;
};

#endif
