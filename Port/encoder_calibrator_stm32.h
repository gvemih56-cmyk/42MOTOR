#ifndef CTRL_STEP_FW_ENCODER_CALIBRATOR_H
#define CTRL_STEP_FW_ENCODER_CALIBRATOR_H

#include "Sensor/Encoder/encoder_calibrator_base.h"   // 引入基类定义

/**
 * @brief 编码器校准类（派生自 EncoderCalibratorBase）
 * @note  该类用于校准绝对值编码器（如 MT6816），
 *        将磁编码器的角度与步进电机实际步数对应起来，
 *        并将结果写入 Flash 保存，以便下次上电自动校正零点。
 *
 * @details
 * 工作流程：
 *   1️⃣ 检测到用户触发（如按键同时按下） → 启动校准。
 *   2️⃣ 电机旋转一整圈，记录编码器的原始数据序列。
 *   3️⃣ 将采集的偏移数据写入 MCU Flash。
 *   4️⃣ 下次启动时读取该校准表，实现角度线性化和零位偏移补偿。
 */
class EncoderCalibrator : public EncoderCalibratorBase
{
public:
    /**
     * @brief 构造函数
     * @param _motor 指向当前 Motor 对象的指针（用于访问电机控制接口）
     */
    explicit EncoderCalibrator(Motor* _motor)
        : EncoderCalibratorBase(_motor)  // 调用基类构造函数
    {}

private:
    /*--------------------------------------------------------------
     * Flash操作函数（重写基类纯虚函数）
     *--------------------------------------------------------------*/

    /**
     * @brief 开始写入Flash前的准备工作
     * @note  可在此解锁Flash、擦除页或准备缓冲区
     */
    void BeginWriteFlash() override;

    /**
     * @brief 写入完成后的收尾工作
     * @note  一般用于重新上锁Flash、清空缓存、打印日志等
     */
    void EndWriteFlash() override;

    /**
     * @brief 清空Flash数据区域
     * @note  用于在重新校准时清除旧的标定表
     */
    void ClearFlash() override;

    /**
     * @brief 追加写入16位数据到Flash
     * @param _data 要写入的16位值
     * @note  用于逐步存储标定数据（通常是角度查找表）
     */
    void WriteFlash16bitsAppend(uint16_t _data) override;
};

#endif
