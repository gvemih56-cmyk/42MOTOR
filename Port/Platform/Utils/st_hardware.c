#include "st_hardware.h"
#include "stm32f103xb.h"

/**
 * @brief  获取MCU唯一序列号（用于自动识别控制板）
 * @note   STM32 每颗芯片在出厂时，都在固定地址(UID_BASE)写入96位唯一ID。
 *         本函数将该96位ID转换成一个64位“序列号”，与STM官方USB DFU算法保持一致，
 *         以保证在“用户模式”和“DFU升级模式”下，设备序列号一致。
 * @retval uint64_t  64位序列号（不同芯片唯一）
 */
uint64_t GetSerialNumber()
{
    // STM32 内置唯一ID寄存器基地址定义为 UID_BASE
    // UID_BASE 地址处存放3个连续的32位值（共96位 = 12字节）
    uint32_t uuid0 = *(uint32_t *)(UID_BASE + 0);   // UID[31:0]   第1段唯一ID
    uint32_t uuid1 = *(uint32_t *)(UID_BASE + 4);   // UID[63:32]  第2段唯一ID
    uint32_t uuid2 = *(uint32_t *)(UID_BASE + 8);   // UID[95:64]  第3段唯一ID

    // 混合算法（与STM DFU引导加载程序一致）：
    // 将第1段和第3段相加，作为高部分，组合第2段的高16位
    uint32_t uuid_mixed_part = uuid0 + uuid2;        // 混合低段与高段

    // 构造最终64位序列号：
    // [63:16] = uuid_mixed_part
    // [15:0]  = uuid1 >> 16（取uuid1的高16位）
    uint64_t serialNumber = ((uint64_t)uuid_mixed_part << 16) | (uint64_t)(uuid1 >> 16);

    return serialNumber;   // 返回组合后的唯一序列号
}
