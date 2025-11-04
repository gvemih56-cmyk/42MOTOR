#ifndef FlashStorage_STM32_h
#define FlashStorage_STM32_h


#if !(defined(STM32F0) || defined(STM32F1) || defined(STM32F2) || defined(STM32F3) || defined(STM32F4) || defined(STM32F7) || \
       defined(STM32L0) || defined(STM32L1) || defined(STM32L4) || defined(STM32H7) || defined(STM32G0) || defined(STM32G4) || \
       defined(STM32WB) || defined(STM32MP1) || defined(STM32L5))
#error This code is intended to run on STM32F/L/H/G/WB/MP1 platform! Please check your Tools->Board setting.
#endif

#define FLASH_STORAGE_STM32_VERSION     "FlashStorage_STM32 v1.1.0"

// Only use this with emulated EEPROM, without integrated EEPROM
#if !defined(DATA_EEPROM_BASE)

#include "emulated_eeprom.h"

/**
 * @brief 模拟EEPROM存储类（基于Flash实现）
 * @note  提供类似Arduino EEPROM的操作接口：
 *         - read(address)       : 读取单字节
 *         - write(address, val) : 写入单字节
 *         - update(address, val): 仅在值改变时才写入
 *         - get(offset, obj)    : 从EEPROM读取一个结构体
 *         - put(offset, obj)    : 将结构体写入EEPROM
 *         - commit()            : 将缓冲区数据写回Flash
 *         - 支持延迟写入机制（_commitASAP控制）
 */
class EEPROM
{
public:

    EEPROM()
        : _initialized(false),     // 是否已初始化（Flash数据读入缓冲区）
          _dirtyBuffer(false),     // 缓冲区是否被修改但未保存
          _commitASAP(true),       // 是否立即写回（true = 实时写入）
          _validEEPROM(true)       // EEPROM数据是否有效（曾经被写入过）
    {}

    /*--------------------------------------------------------------
     * 单字节读取
     *--------------------------------------------------------------*/
    /**
     * @brief 读取一个EEPROM单元（字节）
     * @param address EEPROM地址（偏移）
     * @return 读取的字节值
     */
    uint8_t read(int address)
    {
        if (!_initialized)
            init(); // 若未初始化则先从Flash加载数据

        return eeprom_buffered_read_byte(address); // 调用底层缓冲读取函数
    }

    /*--------------------------------------------------------------
     * 单字节更新（仅当新值不同才写入）
     *--------------------------------------------------------------*/
    /**
     * @brief 更新EEPROM单元（仅当值改变时才写入）
     * @param address 地址偏移
     * @param value 写入的新值
     */
    void update(int address, uint8_t value)
    {
        if (!_initialized)
            init();

        // 仅在数据确实不同的情况下才标记_dirtyBuffer并写入
        if (eeprom_buffered_read_byte(address) != value)
        {
            _dirtyBuffer = true;                    // 缓冲区有修改
            eeprom_buffered_write_byte(address, value); // 写入缓冲
        }
    }

    /*--------------------------------------------------------------
     * 单字节写入（等效于 update）
     *--------------------------------------------------------------*/
    /**
     * @brief 写入一个字节（封装update）
     */
    void write(int address, uint8_t value)
    {
        update(address, value);
    }

    /*--------------------------------------------------------------
     * 模板函数：读取任意类型对象（结构体）
     *--------------------------------------------------------------*/
    /**
     * @brief 从EEPROM读取对象数据到变量中
     * @tparam T 任意数据类型（结构体/类/数组）
     * @param _offset EEPROM偏移地址
     * @param _t 要填充的对象
     * @return 对象引用（已填充）
     */
    template<typename T>
    T &get(int _offset, T &_t)
    {
        if (!_initialized)
            init();

        uint16_t offset = _offset;
        uint8_t* _pointer = (uint8_t*)&_t; // 将结构体地址转为字节指针

        // 按字节拷贝数据从缓冲区到对象
        for (uint16_t count = sizeof(T); count; --count, ++offset)
        {
            *_pointer++ = eeprom_buffered_read_byte(offset);
        }

        return _t;
    }

    /*--------------------------------------------------------------
     * 模板函数：写入任意类型对象（结构体）
     *--------------------------------------------------------------*/
    /**
     * @brief 将对象写入EEPROM（支持延迟或立即写回）
     * @tparam T 任意数据类型
     * @param idx 偏移地址
     * @param t   要写入的对象
     * @return 对象常量引用
     */
    template<typename T>
    const T &put(int idx, const T &t)
    {
        if (!_initialized)
            init();

        uint16_t offset = idx;
        const uint8_t* _pointer = (const uint8_t*)&t;

        // 将对象按字节写入缓冲区
        for (uint16_t count = sizeof(T); count; --count, ++offset)
        {
            eeprom_buffered_write_byte(offset, *_pointer++);
        }

        if (_commitASAP)
        {
            // 立即保存到Flash
            eeprom_buffer_flush();   // 刷新缓冲区 → 写入Flash
            _dirtyBuffer = false;    // 标记为干净
            _validEEPROM = true;     // EEPROM有效
        }
        else
        {
            // 延迟保存，等待 commit() 手动写回
            _dirtyBuffer = true;
        }

        return t;
    }

    /*--------------------------------------------------------------
     * 状态与控制接口
     *--------------------------------------------------------------*/
    /**
     * @brief 检查EEPROM是否曾被写入（有效数据）
     * @return true=有效, false=从未写入
     */
    bool isValid()
    {
        return _validEEPROM;
    }

    /**
     * @brief 手动提交（将缓冲区内容写入Flash）
     * @note  每次写入都会消耗Flash寿命，请谨慎调用。
     */
    void commit()
    {
        if (!_initialized)
            init();

        if (_dirtyBuffer)
        {
            eeprom_buffer_flush();   // 写回Flash
            _dirtyBuffer = false;
            _validEEPROM = true;
        }
    }

    /**
     * @brief 返回EEPROM模拟区大小
     * @return EEPROM字节长度
     */
    uint16_t length()
    { return E2END + 1; }

    /**
     * @brief 设置写入模式（是否实时写入）
     * @param value true=立即写入 false=延迟写入（需手动commit）
     */
    void setCommitASAP(bool value = true)
    { _commitASAP = value; }

    /**
     * @brief 获取当前写入模式
     */
    bool getCommitASAP()
    { return _commitASAP; }

private:

    /**
     * @brief 初始化：将Flash内容加载到缓冲区
     * @note  只在首次调用时执行
     */
    void init()
    {
        eeprom_buffer_fill(); // 从Flash读取内容到缓存区
        _initialized = true;
    }

    // 内部状态标志
    bool _initialized;   // 是否已初始化
    bool _dirtyBuffer;   // 缓冲区是否有未保存内容
    bool _commitASAP;    // 是否实时写入模式
    bool _validEEPROM;   // 数据是否有效
};



#else

#include "EEPROM.h"

#endif    // #if !defined(DATA_EEPROM_BASE)

#endif    //#ifndef FlashAsEEPROM_SAMD_h
