#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#ifdef __cplusplus
extern "C" {
#endif
/*---------------------------- C Scope ---------------------------*/
#include <stdbool.h>
#include "stdint-gcc.h"

    /*--------------------------------------------------------------
     * 枚举类型：配置状态标志
     *--------------------------------------------------------------*/
    /**
     * @brief 配置状态枚举（保存或恢复控制逻辑）
     * @note  用于 Main() 主循环中判断是否写入/恢复EEPROM
     */
    typedef enum configStatus_t
    {
        CONFIG_RESTORE = 0,  // 恢复出厂设置（恢复默认参数并重启）
        CONFIG_OK,           // 当前配置正常有效（无需保存）
        CONFIG_COMMIT        // 配置已修改，等待提交保存到EEPROM
    } configStatus_t;


    /*--------------------------------------------------------------
     * 结构体类型：电机控制板配置参数
     *--------------------------------------------------------------*/
    /**
     * @brief 主控制板配置结构体（保存至EEPROM）
     * @note  包含电机参数、控制参数、功能开关等内容。
     */
    typedef struct Config_t
    {
        configStatus_t configStatus;    // 当前配置状态（见上方枚举）

        uint32_t canNodeId;             // CAN节点号（唯一地址，用于多板通信）
        int32_t encoderHomeOffset;      // 编码器零点偏移量（单位：编码器计数）
        uint32_t defaultMode;           // 默认工作模式（例如：位置控制/速度控制）
        int32_t currentLimit;           // 电流限幅（单位mA，例如1000=1A）
        int32_t velocityLimit;          // 最大速度限制（单位=细分步/秒）
        int32_t velocityAcc;            // 最大加速度限制（单位=细分步/秒²）
        int32_t calibrationCurrent;     // 校准电流（用于编码器归零时提供的驱动力）

        // DCE控制器（等价PID）参数
        int32_t dce_kp;                 // 比例系数（P）
        int32_t dce_kv;                 // 速度前馈系数（V）
        int32_t dce_ki;                 // 积分系数（I）
        int32_t dce_kd;                 // 微分系数（D）

        bool enableMotorOnBoot;         // 上电是否自动启动电机（true=启用）
        bool enableStallProtect;        // 是否开启堵转保护（true=启用）
    } BoardConfig_t;


extern BoardConfig_t boardConfig;


#ifdef __cplusplus
}
/*---------------------------- C++ Scope ---------------------------*/

#include <Platform/Memory/eeprom_interface.h>
#include "Motor/motor.h"


#endif
#endif
