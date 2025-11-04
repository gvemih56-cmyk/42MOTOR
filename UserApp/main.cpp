#include "common_inc.h"
#include "configurations.h"
#include "Platform/Utils/st_hardware.h"
#include <tim.h>


/* Component Definitions -----------------------------------------------------*/
BoardConfig_t boardConfig;  //主控制板配置结构体（保存至EEPROM）
Motor motor;
TB67H450 tb67H450;
MT6816 mt6816;
EncoderCalibrator encoderCalibrator(&motor);  // 编码器校准类（派生自 EncoderCalibratorBase）
Button button1(1, 1000), button2(2, 3000);  //按键
void OnButton1Event(Button::Event _event);
void OnButton2Event(Button::Event _event);
Led statusLed;


/* Main Entry ----------------------------------------------------------------*//* Main Entry ----------------------------------------------------------------*/
void Main()
{
    uint64_t serialNum = GetSerialNumber();       // 获取 MCU 唯一序列号（每块芯片不同）
    uint16_t defaultNodeID = 0;                   // 默认节点号（用于CAN通信识别）

    // 根据芯片序列号为不同控制板分配唯一 Node ID
    switch (serialNum)
    {
        case 431466563640: //J1
            defaultNodeID = 1;                    // 控制板1
            break;
        case 384624576568: //J2
            defaultNodeID = 2;                    // 控制板2
            break;
        case 384290670648: //J3
            defaultNodeID = 3;                    // 控制板3
            break;
        case 431531051064: //J4
            defaultNodeID = 4;                    // 控制板4
            break;
        case 431466760248: //J5
            defaultNodeID = 5;                    // 控制板5
            break;
        case 431484848184: //J6
            defaultNodeID = 6;                    // 控制板6
            break;
        default:
            break;                                // 未识别则保持0（调试状态）
    }


    /*---------- Apply EEPROM Settings ----------*/
    // EEPROM优先级高于 Motor.h 的默认值
    EEPROM eeprom;                                // 创建EEPROM对象（用于读写配置）
    eeprom.get(0, boardConfig);                   // 从地址0读取保存的配置结构体

    if (boardConfig.configStatus != CONFIG_OK)    // 若配置无效，则写入默认配置
    {
        // 使用默认配置参数初始化结构体
        boardConfig = BoardConfig_t{
            .configStatus = CONFIG_OK,                                // 配置状态标志
            .canNodeId = defaultNodeID,                               // 当前控制板CAN节点号
            .encoderHomeOffset = 0,                                   // 编码器零点偏移
            .defaultMode = Motor::MODE_COMMAND_POSITION,              // 默认控制模式（位置控制）
            .currentLimit = 1 * 1000,                                 // 限流值1A（单位mA）
            .velocityLimit = 30 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, // 最大速度30圈/s（按细分步数计）
            .velocityAcc = 100 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS,  // 最大加速度100圈/s²
            .calibrationCurrent = 2000,                               // 编码器校准电流 2A
            .dce_kp = 200,                                            // 控制器比例系数 KP
            .dce_kv = 80,                                             // 控制器速度前馈系数 KV
            .dce_ki = 300,                                            // 控制器积分系数 KI
            .dce_kd = 250,                                            // 控制器微分系数 KD
            .enableMotorOnBoot = false,                               // 上电是否自动启用电机
            .enableStallProtect = false                               // 是否启用堵转保护
        };
        eeprom.put(0, boardConfig);                                   // 将默认配置写入EEPROM保存
    }

    /*---------- 将配置同步到 motor 对象 ----------*/
    motor.config.motionParams.encoderHomeOffset = boardConfig.encoderHomeOffset; // 同步零点偏移
    motor.config.motionParams.ratedCurrent      = boardConfig.currentLimit;       // 同步额定电流
    motor.config.motionParams.ratedVelocity     = boardConfig.velocityLimit;      // 同步最大速度
    motor.config.motionParams.ratedVelocityAcc  = boardConfig.velocityAcc;        // 同步最大加速度
    motor.motionPlanner.velocityTracker.SetVelocityAcc(boardConfig.velocityAcc);  // 设置速度规划器加速度
    motor.motionPlanner.positionTracker.SetVelocityAcc(boardConfig.velocityAcc);  // 设置位置规划器加速度
    motor.config.motionParams.caliCurrent       = boardConfig.calibrationCurrent; // 校准电流
    motor.config.ctrlParams.dce.kp              = boardConfig.dce_kp;             // PID参数 KP
    motor.config.ctrlParams.dce.kv              = boardConfig.dce_kv;             // PID参数 KV
    motor.config.ctrlParams.dce.ki              = boardConfig.dce_ki;             // PID参数 KI
    motor.config.ctrlParams.dce.kd              = boardConfig.dce_kd;             // PID参数 KD
    motor.config.ctrlParams.stallProtectSwitch  = boardConfig.enableStallProtect; // 堵转保护开关


    /*---------------- Init Motor ----------------*/
    motor.AttachDriver(&tb67H450);       // 绑定步进驱动芯片 TB67H450
    motor.AttachEncoder(&mt6816);        // 绑定绝对值磁编码器 MT6816
    motor.controller->Init();            // 初始化控制器（PID/DCE结构）
    motor.driver->Init();                // 初始化驱动器（IO口、PWM等）
    motor.encoder->Init();               // 初始化编码器（SPI读取、零位等）


    /*------------- Init peripherals -------------*/
    button1.SetOnEventListener(OnButton1Event);  // 绑定按键1事件回调函数
    button2.SetOnEventListener(OnButton2Event);  // 绑定按键2事件回调函数


    /*------- Start Close-Loop Control Tick ------*/
    HAL_Delay(100);                       // 稍作延时等待系统稳定
    HAL_TIM_Base_Start_IT(&htim1);        // 启动定时器1中断（100Hz控制周期）
    HAL_TIM_Base_Start_IT(&htim4);        // 启动定时器4中断（20kHz快速环：速度/电流控制）

    // 如果上电时两个按键同时按下 → 进入编码器校准模式
    if (button1.IsPressed() && button2.IsPressed())
        encoderCalibrator.isTriggered = true;


    /*================ Main Loop =================*/
    for (;;)
    {
        encoderCalibrator.TickMainLoop(); // 周期执行编码器校准任务（若被触发）

        // 检查配置状态：若被标记为 COMMIT → 写入EEPROM保存
        if (boardConfig.configStatus == CONFIG_COMMIT)
        {
            boardConfig.configStatus = CONFIG_OK; // 标记为已生效
            eeprom.put(0, boardConfig);           // 保存当前配置到EEPROM
        }
        // 若被标记为 RESTORE → 恢复默认并重启系统
        else if (boardConfig.configStatus == CONFIG_RESTORE)
        {
            eeprom.put(0, boardConfig);           // 先保存当前状态
            HAL_NVIC_SystemReset();               // 系统软复位
        }
    }
}



/*====================================================================
 * 定时器事件回调函数定义
 *====================================================================*/

/**
 * @brief  TIM1 更新中断回调函数（周期 100Hz）
 * @note   用于执行低频任务：
 *         - 按键扫描与事件判断
 *         - 状态LED刷新显示
 */
extern "C" void Tim1Callback100Hz()
{
    // 清除 TIM1 更新中断标志
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

    // 每 10ms 调用一次按键与状态灯刷新函数
    button1.Tick(10);                                 // 扫描按键1（消抖 + 事件识别）
    button2.Tick(10);                                 // 扫描按键2
    statusLed.Tick(10, motor.controller->state);      // LED 状态指示（随电机状态变化）
}


/**
 * @brief  TIM4 更新中断回调函数（周期 20kHz）
 * @note   用于执行高频任务：
 *         - 编码器校准过程采样（如果被触发）
 *         - 电机闭环控制算法执行（正常模式）
 */
extern "C" void Tim4Callback20kHz()
{
    // 清除 TIM4 更新中断标志
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);

    // 判断当前是否处于编码器校准模式
    if (encoderCalibrator.isTriggered)
        encoderCalibrator.Tick20kHz();   // 调用编码器校准高速任务
    else
        motor.Tick20kHz();               // 否则执行电机闭环控制任务
}


/*====================================================================
 * 按键事件回调函数
 *====================================================================*/
/**
 * @brief  按键1事件回调函数
 * @param  _event 按键事件类型（UP、DOWN、CLICK、LONG_PRESS）
 * @note   该函数在按键状态变化时由 ButtonBase 自动触发。
 */
void OnButton1Event(Button::Event _event)
{
    switch (_event)
    {
        case ButtonBase::UP:
            // 松开按键（一般不做处理）
            break;

        case ButtonBase::DOWN:
            // 按下按键（短按开始，预留扩展）
            break;

        case ButtonBase::LONG_PRESS:
            // 长按1号按键：系统复位
            HAL_NVIC_SystemReset();
            break;

        case ButtonBase::CLICK:
            // 单击：切换电机模式（启动/停止）
            if (motor.controller->modeRunning != Motor::MODE_STOP)
            {
                // 若电机正在运行 → 停止并保存当前模式
                boardConfig.defaultMode = motor.controller->modeRunning;  // 记录上次模式
                motor.controller->requestMode = Motor::MODE_STOP;         // 请求停止
            }
            else
            {
                // 若当前电机已停止 → 恢复上次模式运行
                motor.controller->requestMode =
                    static_cast<Motor::Mode_t>(boardConfig.defaultMode);  // 恢复默认模式
            }
            break;
    }
}

/**
 * @brief  按键2事件回调函数
 * @param  _event 按键事件类型（UP、DOWN、CLICK、LONG_PRESS）
 * @note   按键2的功能：
 *         - 长按：使电机归零（根据当前控制模式清零目标值）
 *         - 单击：清除堵转保护标志（恢复运行）
 */
void OnButton2Event(Button::Event _event)
{
    switch (_event)
    {
        case ButtonBase::UP:
            // 按键松开 → 无操作（保留）
            break;

        case ButtonBase::DOWN:
            // 按键按下 → 无操作（可扩展）
            break;

        case ButtonBase::LONG_PRESS:
            /**
             * @brief 长按按键2：归零操作
             * @details
             * 根据当前电机控制模式，执行不同的“目标置零”：
             *  - 电流模式：电流设为0 → 停止通电
             *  - 速度模式：目标速度设为0 → 停止旋转
             *  - 位置模式：目标位置设为0 → 回归零点
             *  - 其他模式（轨迹、步进、停止）：不处理
             */
            switch (motor.controller->modeRunning)
            {
                // ---- 电流控制模式 ----
            case Motor::MODE_COMMAND_CURRENT:
            case Motor::MODE_PWM_CURRENT:
                    motor.controller->SetCurrentSetPoint(0);
                    break;

                    // ---- 速度控制模式 ----
            case Motor::MODE_COMMAND_VELOCITY:
            case Motor::MODE_PWM_VELOCITY:
                    motor.controller->SetVelocitySetPoint(0);
                    break;

                    // ---- 位置控制模式 ----
            case Motor::MODE_COMMAND_POSITION:
            case Motor::MODE_PWM_POSITION:
                    motor.controller->SetPositionSetPoint(0);
                    break;

                    // ---- 其他模式不响应归零 ----
            case Motor::MODE_COMMAND_Trajectory:
            case Motor::MODE_STEP_DIR:
            case Motor::MODE_STOP:
                    break;
            }
            break;

        case ButtonBase::CLICK:
            /**
             * @brief 单击按键2：清除堵转标志
             * @details
             * 若系统检测到堵转（stall）后自动进入保护状态，
             * 可通过单击按钮2来清除标志，重新使能运行。
             */
            motor.controller->ClearStallFlag();
            break;
    }
}
