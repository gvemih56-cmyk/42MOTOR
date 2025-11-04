#ifndef CTRL_STEP_FW_MT6816_STM32_H
#define CTRL_STEP_FW_MT6816_STM32_H

#include "Sensor/Encoder/mt6816_base.h"   // 引入MT6816编码器基类（通用逻辑部分）

/**
 * @brief STM32 平台 MT6816 磁编码器类
 * @note  MT6816 是一款 16bit 分辨率的绝对值磁编码器，
 *        可通过 SPI / PWM / ABZ 等接口输出角度数据。
 *
 *        本类继承自 MT6816Base，负责 STM32 平台的 SPI 硬件通信实现。
 */
class MT6816 : public MT6816Base
{
public:
    /**
     * @brief 构造函数
     * @note  这里调用基类构造函数，传入校准数据存储起始地址。
     *
     * @details
     * STM32F103CBT6 的 Flash 空间为 128KB（0x08000000 ~ 0x08020000）
     * 程序中约定最后 33KB（32KB 校准数据 + 1KB 用户参数）为 EEPROM 区。
     * 校准数据（encoder calibration table）起始地址为：
     *     0x08017C00 = 96KB + 31KB偏移处
     * 即从 Flash 尾部 33KB 区域的开头开始。
     */
    explicit MT6816()
        : MT6816Base((uint16_t*)(0x08017C00))  // 传入Flash校准表地址
    {}

private:
    /*--------------------------------------------------------------
     * SPI接口函数（重写基类虚函数）
     *--------------------------------------------------------------*/

    /**
     * @brief 初始化 SPI 外设
     * @note  用于配置 STM32 的 SPI 接口（时钟极性、相位、速率等）
     *        使能 GPIO 片选信号，并确保通信可用。
     */
    void SpiInit() override;

    /**
     * @brief SPI 发送并读取 16 位数据
     * @param _data 要发送的数据
     * @return 读取到的 16bit 返回值（MT6816角度或状态数据）
     * @note  通过 SPI 与 MT6816 通信，通常一次读16位。
     *        高字节和低字节分别为角度高/低8位。
     */
    uint16_t SpiTransmitAndRead16Bits(uint16_t _data) override;
};

#endif
