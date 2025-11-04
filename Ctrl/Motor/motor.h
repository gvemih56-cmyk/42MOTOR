#ifndef CTRL_STEP_FW_MOTOR_H
#define CTRL_STEP_FW_MOTOR_H

#include "Motor/motion_planner.h"          // 运动规划器模块（轨迹生成/速度规划）
#include "Sensor/Encoder/encoder_base.h"   // 编码器基类（读取角度/位置反馈）
#include "Driver/driver_base.h"            // 驱动器基类（控制步进电机的PWM或方向脉冲）

/**
 * @brief 电机类 Motor
 * @note  该类封装了整个步进电机闭环控制系统，包括：
 *         - 电机配置（Config）
 *         - 控制器（Controller）
 *         - 驱动器（Driver）
 *         - 编码器（Encoder）
 *         - 运动规划器（MotionPlanner）
 */
class Motor
{
public:
    /**********************************************************
     * 构造函数：初始化默认配置参数
     **********************************************************/
    Motor() :
        controller(&controllerInstance)     // 初始化控制器对象（关联当前Motor实例）
    {
        /****************** 默认配置区 *******************/
        // --- 运动学参数初始化 ---
        config.motionParams.encoderHomeOffset = 0;                  // 编码器零点偏移
        config.motionParams.caliCurrent = 2000;                     // 校准电流 = 2000mA
        config.motionParams.ratedCurrent = 1000;                    // 额定电流 = 1000mA
        config.motionParams.ratedCurrentAcc = 2 * 1000;             // 电流加速度 = 2000mA/s
        config.motionParams.ratedVelocity = 30 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS; // 最大速度 = 30圈/s
        config.motionParams.ratedVelocityAcc = 1000 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS; // 加速度 = 1000圈/s²

        // --- 控制参数初始化 ---
        config.ctrlParams.stallProtectSwitch = false;               // 默认关闭堵转保护
        config.ctrlParams.pid =
            Controller::PID_t{                                      // PID控制器默认参数
                .kp = 5,
                .ki = 30,
                .kd = 0
            };
        config.ctrlParams.dce =
            Controller::DCE_t{                                      // DCE控制器默认参数（类似扩展PID）
                .kp = 200,
                .kv = 80,
                .ki = 300,
                .kd = 250
            };

        /*****************************************************/

        // 将配置指针绑定给运动规划器与控制器
        motionPlanner.AttachConfig(&config.motionParams);           // 让运动规划器使用运动参数配置
        controller->AttachConfig(&config.ctrlParams);               // 让控制器使用控制参数配置
    }

    /*--------------------------------------------------------------
     * 电机常量定义
     *--------------------------------------------------------------*/
    const int32_t MOTOR_ONE_CIRCLE_HARD_STEPS = 200;  // 一圈200步（标准1.8°步进）
    const int32_t SOFT_DIVIDE_NUM = 256;              // 细分数（软细分）
    const int32_t MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS =
        MOTOR_ONE_CIRCLE_HARD_STEPS * SOFT_DIVIDE_NUM; // 每圈总步数 = 200 × 256 = 51200步

    /*--------------------------------------------------------------
     * 电机模式定义（控制目标）
     *--------------------------------------------------------------*/
    typedef enum
    {
        MODE_STOP,                 // 停止模式
        MODE_COMMAND_POSITION,     // 位置命令模式
        MODE_COMMAND_VELOCITY,     // 速度命令模式
        MODE_COMMAND_CURRENT,      // 电流命令模式
        MODE_COMMAND_Trajectory,   // 轨迹模式（运动规划控制）
        MODE_PWM_POSITION,         // PWM控制 - 位置模式
        MODE_PWM_VELOCITY,         // PWM控制 - 速度模式
        MODE_PWM_CURRENT,          // PWM控制 - 电流模式
        MODE_STEP_DIR,             // STEP/DIR 接口模式（外部脉冲控制）
    } Mode_t;

    /*--------------------------------------------------------------
     * 电机状态定义
     *--------------------------------------------------------------*/
    typedef enum
    {
        STATE_STOP,      // 停止中
        STATE_FINISH,    // 动作完成
        STATE_RUNNING,   // 正在运行
        STATE_OVERLOAD,  // 过载
        STATE_STALL,     // 堵转
        STATE_NO_CALIB   // 未校准
    } State_t;


    /**********************************************************
     * 内嵌类：Controller 控制器
     * 用于实现 PID / DCE 算法与状态管理
     **********************************************************/
    class Controller
    {
    public:
        friend Motor;    // 允许 Motor 类访问其私有成员

        /*---------------- PID控制结构 ----------------*/
        typedef struct
        {
            bool kpValid, kiValid, kdValid;  // 参数有效标志
            int32_t kp, ki, kd;              // PID 系数
            int32_t vError, vErrorLast;      // 当前误差与上次误差
            int32_t outputKp, outputKi, outputKd;  // 各分量输出
            int32_t integralRound;           // 积分部分循环数
            int32_t integralRemainder;       // 积分余数
            int32_t output;                  // 最终输出量
        } PID_t;

        /*---------------- DCE控制结构 ----------------*/
        typedef struct
        {
            int32_t kp, kv, ki, kd;          // 控制参数（含速度前馈 kv）
            int32_t pError, vError;          // 位置误差与速度误差
            int32_t outputKp, outputKi, outputKd;
            int32_t integralRound;
            int32_t integralRemainder;
            int32_t output;                  // 控制器输出（PWM/电流命令）
        } DCE_t;

        /*---------------- 控制参数配置 ----------------*/
        typedef struct
        {
            PID_t pid;                       // PID控制器参数
            DCE_t dce;                       // DCE控制器参数
            bool stallProtectSwitch;         // 堵转保护开关
        } Config_t;


        /*---------------- 构造函数 ----------------*/
        explicit Controller(Motor* _context)
        {
            context = _context;              // 绑定所属电机实例
            requestMode = MODE_STOP;         // 请求模式初始化为停止
            modeRunning = MODE_STOP;         // 当前运行模式初始化为停止
        }

        /*---------------- 公有成员 ----------------*/
        Config_t* config = nullptr;          // 指向控制参数配置的指针
        Mode_t requestMode;                  // 外部请求的模式（目标）
        Mode_t modeRunning;                  // 当前实际运行模式
        State_t state = STATE_STOP;          // 当前状态
        bool isStalled = false;              // 堵转标志

        /*---------------- 控制器接口函数 ----------------*/
        void Init();                         // 初始化控制器（设定初值等）
        void SetCtrlMode(Mode_t _mode);      // 设置控制模式
        void SetCurrentSetPoint(int32_t _cur);  // 设置目标电流
        void SetVelocitySetPoint(int32_t _vel); // 设置目标速度
        void SetPositionSetPoint(int32_t _pos); // 设置目标位置
        bool SetPositionSetPointWithTime(int32_t _pos, float _time); // 设置目标位置+时间
        float GetPosition(bool _isLap = false);  // 获取当前位置（是否按圈计）
        float GetVelocity();                     // 获取当前速度
        float GetFocCurrent();                   // 获取当前电流（闭环电流值）
        void AddTrajectorySetPoint(int32_t _pos, int32_t _vel);  // 添加轨迹目标点
        void SetDisable(bool _disable);          // 使能/禁用电机
        void SetBrake(bool _brake);              // 设置刹车状态
        void ApplyPosAsHomeOffset();             // 将当前位置作为零点
        void ClearStallFlag();                   // 清除堵转标志

    private:
        /*---------------- 内部变量 ----------------*/
        Motor* context;                // 指向所属电机对象
        int32_t realLapPosition{};     // 实际圈数位置
        int32_t realLapPositionLast{};
        int32_t realPosition{};        // 实际绝对位置
        int32_t realPositionLast{};
        int32_t estVelocity{};         // 估算速度
        int32_t estVelocityIntegral{};
        int32_t estLeadPosition{};
        int32_t estPosition{};
        int32_t estError{};
        int32_t focCurrent{};
        int32_t goalPosition{};        // 目标位置
        int32_t goalVelocity{};        // 目标速度
        int32_t goalCurrent{};         // 目标电流
        bool goalDisable{};
        bool goalBrake{};
        int32_t softPosition{};
        int32_t softVelocity{};
        int32_t softCurrent{};
        bool softDisable{};
        bool softBrake{};
        bool softNewCurve{};
        int32_t focPosition{};
        uint32_t stalledTime{};        // 堵转持续时间
        uint32_t overloadTime{};       // 过载持续时间
        bool overloadFlag{};           // 过载标志

        /*---------------- 内部函数 ----------------*/
        void AttachConfig(Config_t* _config);        // 绑定配置结构体
        void CalcCurrentToOutput(int32_t current);   // 电流控制算法
        void CalcPidToOutput(int32_t _speed);        // PID控制算法
        void CalcDceToOutput(int32_t _location, int32_t _speed); // DCE控制算法
        void ClearIntegral() const;                  // 清除积分项
        static int32_t CompensateAdvancedAngle(int32_t _vel); // 前馈角度补偿
    };

    /*--------------------------------------------------------------
     * 电机整体配置结构体
     *--------------------------------------------------------------*/
    struct Config_t
    {
        MotionPlanner::Config_t motionParams{};  // 运动规划参数（速度、加速度、零点偏移等）
        Controller::Config_t ctrlParams{};       // 控制参数（PID/DCE等）
    };
    Config_t config;                             // 当前配置实例

    /*--------------------------------------------------------------
     * 电机子模块对象
     *--------------------------------------------------------------*/
    MotionPlanner motionPlanner;                 // 运动规划器对象
    Controller* controller = nullptr;            // 控制器指针
    EncoderBase* encoder = nullptr;              // 编码器对象指针
    DriverBase* driver = nullptr;                // 驱动器对象指针


    /*--------------------------------------------------------------
     * 主接口函数
     *--------------------------------------------------------------*/
    void Tick20kHz();                            // 主控制周期函数（20kHz定时器调用）
    void AttachEncoder(EncoderBase* _encoder);   // 绑定编码器对象
    void AttachDriver(DriverBase* _driver);      // 绑定驱动器对象

private:
    Controller controllerInstance = Controller(this); // 控制器实例（绑定自身）

    void CloseLoopControlTick();                 // 闭环控制周期函数
};

#endif
