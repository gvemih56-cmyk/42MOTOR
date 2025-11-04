/*
#ifndef CTRL_STEP_FW_MT6816_H
#define CTRL_STEP_FW_MT6816_H

#include "encoder_base.h"
#include <cstdint>

class MT6816Base : public EncoderBase
{
public:
    explicit MT6816Base(uint16_t* _quickCaliDataPtr) :
        quickCaliDataPtr(_quickCaliDataPtr),
        spiRawData(SpiRawData_t{0})
    {
    }


    bool Init() override;
    uint16_t UpdateAngle() override;  // Get current rawAngle (rad)
    bool IsCalibrated() override;


private:
    typedef struct
    {
        uint16_t rawData;       // SPI raw 16bits data
        uint16_t rawAngle;      // 14bits rawAngle in rawData
        bool noMagFlag;
        bool checksumFlag;
    } SpiRawData_t;


    SpiRawData_t spiRawData;
    uint16_t* quickCaliDataPtr;
    uint16_t dataTx[2];
    uint16_t dataRx[2];
    uint8_t hCount;


    /***** Port Specified Implements ****#1#
    virtual void SpiInit();

    virtual uint16_t SpiTransmitAndRead16Bits(uint16_t _dataTx);

};

#endif
*/










#ifndef CTRL_STEP_FW_MT6816_H
#define CTRL_STEP_FW_MT6816_H

#include "encoder_base.h"
#include <cstdint>

/**
 * @brief MT6816 磁编码器基类
 * @note  封装了解析逻辑（SPI读写帧、奇偶校验、角度计算）
 *        但不包含具体的SPI硬件实现，需要子类重写。
 */
class MT6816Base : public EncoderBase
{
public:
    explicit MT6816Base(uint16_t* _quickCaliDataPtr)
        : quickCaliDataPtr(_quickCaliDataPtr),
          spiRawData(SpiRawData_t{0})
    {}

    bool Init() override;
    uint16_t UpdateAngle() override;   // 获取角度
    bool IsCalibrated() override;      // 校准状态检查

protected:
    /***** 平台相关接口（由子类实现） *****/
    virtual void SpiInit() = 0;                              // 初始化 SPI 外设
    virtual uint16_t SpiTransmitAndRead16Bits(uint16_t tx) = 0;  // SPI 收发 16bit

private:
    typedef struct
    {
        uint16_t rawData;       // SPI原始16位数据
        uint16_t rawAngle;      // 提取的14位角度
        bool noMagFlag;         // 无磁信号标志
        bool checksumFlag;      // 校验位标志
    } SpiRawData_t;

    SpiRawData_t spiRawData;
    uint16_t* quickCaliDataPtr; // 指向快速校准查表
    uint16_t dataTx[2];
    uint16_t dataRx[2];
    uint8_t hCount;
};

#endif // CTRL_STEP_FW_MT6816_H


