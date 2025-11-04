#ifndef CTRL_STEP_FW_BUTTON_STM32_H
#define CTRL_STEP_FW_BUTTON_STM32_H

#include "button_base.h"   // 引入按键基类定义（包含通用逻辑与回调机制）

/**
 * @brief STM32 平台按键类（继承自 ButtonBase）
 * @note  该类实现了基类的硬件接口函数 ReadButtonPinIO()
 *        用于从 GPIO 引脚读取按键状态。
 *        逻辑层面的消抖、事件触发由 ButtonBase 实现。
 */
class Button : public ButtonBase
{
public:
    /*--------------------------------------------------------------
     * 构造函数1：仅传入按键ID
     *--------------------------------------------------------------*/
    explicit Button(uint8_t _id)
        : ButtonBase(_id)       // 调用父类构造函数，传入按键编号
    {}

    /*--------------------------------------------------------------
     * 构造函数2：传入按键ID与长按判定时间
     *--------------------------------------------------------------*/
    Button(uint8_t _id, uint32_t _longPressTime)
        : ButtonBase(_id, _longPressTime) // 同时设置长按判定时间(ms)
    {}

    /*--------------------------------------------------------------
     * 按键状态检测接口
     *--------------------------------------------------------------*/
    /**
     * @brief 判断按键是否被按下
     * @return true=按下，false=松开
     */
    bool IsPressed();

private:
    /*--------------------------------------------------------------
     * 硬件层接口函数（重写父类的纯虚函数）
     *--------------------------------------------------------------*/
    /**
     * @brief 从指定按键ID读取IO状态
     * @param _id 按键编号
     * @return true=按下, false=未按
     * @note  该函数需要结合具体STM32的GPIO输入实现
     */
    bool ReadButtonPinIO(uint8_t _id) override;
};

#endif
