#include <stdint.h>

#include "cart_adapter.h"
#include "main.h"
#include "lk_special_hal.h"

/* AD 输入保持上拉，输出保持原推挽速度；常量调用可直接展开。 */
#define CART_AD_INPUT() do { \
    VOLATILE_32(GPIOB->CRH) = 0x88888888; \
    VOLATILE_32(GPIOB->CRL) = 0x88888888; \
    VOLATILE_32(GPIOB->ODR) = 0x0000ffff; \
} while (0)

#define CART_AD_OUTPUT() do { \
    VOLATILE_32(GPIOB->CRH) = 0x33333333; \
    VOLATILE_32(GPIOB->CRL) = 0x33333333; \
} while (0)

void NO_OPTIMIZE cart_setDirection_ad(uint8_t dir)
{
    if (dir == 0) {
        CART_AD_INPUT();
    } else {
        CART_AD_OUTPUT();
    }
}

void NO_OPTIMIZE cart_setDirection_a(uint8_t dir)
{
    if (dir == 0) {
        VOLATILE_32(GPIOA->CRL) = 0x88888888;   // 7-0 上下拉输入
        VOLATILE_32(GPIOA->BSRR) = 0x000000ff;  // 7-0 上拉
    } else {
        VOLATILE_32(GPIOA->CRL) = 0x33333333;  // 7-0 推挽输出, 50mhz
    }
}

static inline uint16_t NO_OPTIMIZE cart_readBus_ad()
{
    return (uint16_t)(VOLATILE_32(GPIOB->IDR) & 0x0000FFFF);
}

static inline uint8_t NO_OPTIMIZE cart_readBus_a()
{
    return (uint8_t)(VOLATILE_32(GPIOA->IDR) & 0x000000FF);
}

void NO_OPTIMIZE cart_writeBus_ad(uint16_t value)
{
    VOLATILE_32(GPIOB->ODR) = value;
}

void NO_OPTIMIZE cart_writeBus_a(uint8_t value)
{
    uint32_t temp = VOLATILE_32(GPIOA->ODR);
    VOLATILE_32(GPIOA->ODR) = (temp & 0xFFFFFF00u) | value;
}

void cart_romRead_begin(uint32_t addr)
{
    // latch base address
    cart_setDirection_a(1);
    cart_setDirection_ad(1);

    cart_writeBus_a((addr & 0x00ff0000) >> 16);
    cart_writeBus_ad((addr & 0x0000ffff));

    VOLATILE_32(cs1_GPIO_Port->BSRR) = cs1_Pin << 16;  // cs1=0 126ns
    TIMING_DELAY();

    // read bus
    cart_setDirection_ad(0);
}

void cart_romRead_continue(uint16_t *buf, uint16_t len)
{
    for (int i = 0; i < len; i++) {
        VOLATILE_32(rd_GPIO_Port->BSRR) = rd_Pin << 16;  // rd=0 126ns
        TIMING_DELAY();                                  // Ensure timing requirements

        // tOE >25ns, tACC >110ns
        *buf = cart_readBus_ad();

        VOLATILE_32(rd_GPIO_Port->BSRR) = rd_Pin;  // rd=1 126ns
        TIMING_DELAY();                            // Ensure timing requirements

        buf++;
    }

}

bool cart_romRead_verify(const uint8_t *expected, uint16_t *buf, uint16_t words)
{
    uint32_t differences = 0;
    uint32_t remaining = words;
    uint32_t actual, reference;

    /* 将取比较字和计数排在 /RD 拉低后的等待里。仍读取整个批次，
       逐字存下样本，只有不匹配时才用它定位第一个错字节。 */
    __asm volatile(
        "cmp %[remaining], #0\n\t"
        "beq 2f\n\t"
        "1:\n\t"
        "str %[rd_low], [%[strobe]]\n\t"
        "ldrh %[reference], [%[expected]], #2\n\t"
        "subs %[remaining], %[remaining], #1\n\t"
        ".rept 9\n\tnop\n\t.endr\n\t"
        "ldr %[actual], [%[gpio]]\n\t"
        "strh %[actual], [%[buf]], #2\n\t"
        "eor %[actual], %[actual], %[reference]\n\t"
        "orr %[differences], %[differences], %[actual]\n\t"
        "nop\n\t"
        "str %[rd_high], [%[strobe]]\n\t"
        ".rept 6\n\tnop\n\t.endr\n\t"
        "bne 1b\n\t"
        "2:"
        : [actual] "=&r" (actual), [reference] "=&r" (reference),
          [remaining] "+&r" (remaining), [expected] "+&r" (expected),
          [buf] "+&r" (buf), [differences] "+&r" (differences)
        : [gpio] "r" (&GPIOB->IDR), [strobe] "r" (&rd_GPIO_Port->BSRR),
          [rd_low] "r" ((uint32_t)rd_Pin << 16), [rd_high] "r" ((uint32_t)rd_Pin)
        : "cc", "memory");

    /* GPIOB->IDR 只取低 16 位卡带数据。 */
    return (uint16_t)differences == 0;
}

/* A16–A23 由整页轮询共用。每次仍锁存低位地址并发出独立的 /CS 和 /RD。 */
__RAM_FUNC uint16_t cart_romPoll_read(uint16_t address)
{
    CART_AD_OUTPUT();
    VOLATILE_32(GPIOB->ODR) = address;
    __asm volatile(".rept 6\n\tnop\n\t.endr" ::: "memory");
    CART_CS1(0);
    TIMING_DELAY();

    CART_AD_INPUT();
    TIMING_DELAY();
    CART_RD(0);
    /* 直接采样时保留原函数调用与加载 GPIO 地址所占的等待。 */
    __asm volatile(".rept 8\n\tnop\n\t.endr" ::: "memory");
    uint16_t value = (uint16_t)VOLATILE_32(GPIOB->IDR);
    __asm volatile(".rept 4\n\tnop\n\t.endr" ::: "memory");
    CART_RD(1);
    TIMING_DELAY();
    CART_CS1(1);
    TIMING_DELAY();
    return value;
}

__RAM_FUNC __attribute__((noinline))
uint32_t cart_romRead_crc32(uint32_t crc, const uint32_t *table, uint16_t words)
{
    uint32_t remaining = words;
    uint32_t sample, index;

    /* 每次 /RD 仍只取一个字，低、高字节依次进入 IEEE CRC32。
       低字节查表覆盖采样后的低电平，高字节查表放在 /RD 高电平期间。 */
    __asm volatile(
        "cmp %[remaining], #0\n\t"
        "beq 2f\n\t"
        "1:\n\t"
        "str %[rd_low], [%[strobe]]\n\t"
        ".rept 10\n\tnop\n\t.endr\n\t"
        "ldr %[sample], [%[gpio]]\n\t"
        "eor %[index], %[crc], %[sample]\n\t"
        "uxtb %[index], %[index]\n\t"
        "ldr %[index], [%[table], %[index], lsl #2]\n\t"
        "eor %[crc], %[index], %[crc], lsr #8\n\t"
        "str %[rd_high], [%[strobe]]\n\t"
        "eor %[index], %[crc], %[sample], lsr #8\n\t"
        "uxtb %[index], %[index]\n\t"
        "ldr %[index], [%[table], %[index], lsl #2]\n\t"
        "eor %[crc], %[index], %[crc], lsr #8\n\t"
        ".rept 4\n\tnop\n\t.endr\n\t"
        "subs %[remaining], %[remaining], #1\n\t"
        "bne 1b\n\t"
        "2:"
        : [sample] "=&r" (sample), [index] "=&r" (index),
          [crc] "+&r" (crc), [remaining] "+&r" (remaining)
        : [table] "r" (table), [gpio] "r" (&GPIOB->IDR),
          [strobe] "r" (&rd_GPIO_Port->BSRR), [rd_low] "r" ((uint32_t)rd_Pin << 16),
          [rd_high] "r" ((uint32_t)rd_Pin)
        : "cc", "memory");
    return crc;
}

void cart_romRead_end(void)
{
    // 释放 ROM 总线
    VOLATILE_32(cs1_GPIO_Port->BSRR) = cs1_Pin;  // cs1=1 126ns
    TIMING_DELAY();
    cart_setDirection_a(0);
}

void cart_romRead(uint32_t addr, uint16_t *buf, uint16_t len)
{
    cart_romRead_begin(addr);
    cart_romRead_continue(buf, len);
    cart_romRead_end();
}

static inline void rom_write_select(uint32_t address)
{
    cart_writeBus_a((address & 0x00ff0000) >> 16);
    cart_writeBus_ad(address & 0x0000ffff);
    CART_CS1(0);
    TIMING_DELAY();
}

static inline void rom_write_pulse(void)
{
    CART_WR(0);
    TIMING_DELAY();
    CART_WR(1);
    TIMING_DELAY();
}

static inline void rom_write_deselect(void)
{
    CART_CS1(1);
    TIMING_DELAY();
}

void cart_romWrite(uint32_t addr, const uint16_t *buf, uint16_t len)
{
    cart_romWrite_setup();
    rom_write_select(addr);
    for (uint16_t i = 0; i < len; ++i) {
        cart_writeBus_ad(*buf++);
        rom_write_pulse();
    }
    rom_write_deselect();
    cart_romWrite_finish();
}

static inline void amd_page_select(uint32_t address)
{
    uint32_t high = (address >> 16) & 0xffU;
    /* BSRR 只更新 A16–A23，控制脚保持原样。 */
    GPIOA->BSRR = high | ((0xffU ^ high) << 16);
    VOLATILE_32(GPIOB->ODR) = (uint16_t)address;
    __asm volatile(".rept 6\n\tnop\n\t.endr" ::: "memory");
    CART_CS1(0);
    TIMING_DELAY();
}

void cart_romWrite_command(uint32_t addr, uint16_t value)
{
    amd_page_select(addr);
    VOLATILE_32(GPIOB->ODR) = value;
    __asm volatile(".rept 6\n\tnop\n\t.endr" ::: "memory");
    rom_write_pulse();
    rom_write_deselect();
}

__RAM_FUNC void cart_romWrite_bytes(uint32_t addr, const uint8_t *buf, uint16_t words)
{
    amd_page_select(addr);

    uint32_t pairs = words / 2U;
    uint32_t value;
    /* 第一字的 /WR 低电平期间预取第二字，第二字期间更新循环计数。
       两次写脉冲和数据建立间隔都保留，与单字尾部使用相同总线事务。 */
    __asm volatile(
        "cmp %[pairs], #0\n\t"
        "beq 2f\n\t"
        "1:\n\t"
        "ldrh %[value], [%[buf]], #2\n\t"
        "str %[value], [%[gpio]]\n\t"
        ".rept 6\n\tnop\n\t.endr\n\t"
        "str %[wr_low], [%[strobe]]\n\t"
        "ldrh %[value], [%[buf]], #2\n\t"
        ".rept 5\n\tnop\n\t.endr\n\t"
        "str %[wr_high], [%[strobe]]\n\t"
        ".rept 6\n\tnop\n\t.endr\n\t"
        "str %[value], [%[gpio]]\n\t"
        ".rept 6\n\tnop\n\t.endr\n\t"
        "str %[wr_low], [%[strobe]]\n\t"
        "subs %[pairs], %[pairs], #1\n\t"
        ".rept 5\n\tnop\n\t.endr\n\t"
        "str %[wr_high], [%[strobe]]\n\t"
        ".rept 6\n\tnop\n\t.endr\n\t"
        "bne 1b\n\t"
        "2:"
        : [value] "=&r" (value), [buf] "+&r" (buf), [pairs] "+&r" (pairs)
        : [gpio] "r" (&GPIOB->ODR), [strobe] "r" (&wr_GPIO_Port->BSRR),
          [wr_low] "r" ((uint32_t)wr_Pin << 16), [wr_high] "r" ((uint32_t)wr_Pin)
        : "cc", "memory");

    if (words & 1U) {
        uint16_t tail;
        __builtin_memcpy(&tail, buf, sizeof(tail));
        VOLATILE_32(GPIOB->ODR) = tail;
        __asm volatile(".rept 6\n\tnop\n\t.endr" ::: "memory");
        rom_write_pulse();
    }
    rom_write_deselect();
}

void cart_ramRead(uint16_t addr, uint8_t *buf, uint16_t len)
{
    cart_setDirection_a(0);
    cart_setDirection_ad(1);

    VOLATILE_32(cs2_GPIO_Port->BSRR) = cs2_Pin << 16;  // cs2=0 126ns
    TIMING_DELAY();

    // read bus
    for (int i = 0; i < len; i++) {
        cart_writeBus_ad(addr);

        VOLATILE_32(rd_GPIO_Port->BSRR) = rd_Pin << 16;  // rd=0 126ns
        TIMING_DELAY();                                  // address to dq 105ns, oe to dq 25ns

        *buf = cart_readBus_a();

        VOLATILE_32(rd_GPIO_Port->BSRR) = rd_Pin;  // rd=1 126ns
        TIMING_DELAY();

        addr++;
        buf++;
    }

    // release bus
    VOLATILE_32(cs2_GPIO_Port->BSRR) = cs2_Pin;  // cs2=1 126ns
    TIMING_DELAY();
    cart_setDirection_ad(0);
}

void cart_ramWrite(uint16_t addr, const uint8_t *buf, uint16_t len)
{
    cart_setDirection_a(1);
    cart_setDirection_ad(1);

    VOLATILE_32(cs2_GPIO_Port->BSRR) = cs2_Pin << 16;  // cs2=0 126ns
    TIMING_DELAY();

    // write bus
    for (int i = 0; i < len; i++) {
        cart_writeBus_ad(addr);
        cart_writeBus_a(*buf);

        VOLATILE_32(wr_GPIO_Port->BSRR) = wr_Pin << 16;
        TIMING_DELAY();  // address hold 70ns, data setup 20ns, write cycle 105ns
        VOLATILE_32(wr_GPIO_Port->BSRR) = wr_Pin;
        TIMING_DELAY();

        addr++;
        buf++;
    }

    // release bus
    VOLATILE_32(cs2_GPIO_Port->BSRR) = cs2_Pin;
    TIMING_DELAY();
    cart_setDirection_a(0);
    cart_setDirection_ad(0);
}

//
// for gb/gbc
//

static inline __attribute__((always_inline)) void gbc_read(uint16_t addr, uint8_t *buf, uint16_t len,
                     uint8_t method, bool pulse_cs)
{
    cart_setDirection_a(0);
    cart_setDirection_ad(1);

    if (!pulse_cs) CART_CS1(0);
    TIMING_DELAY();

    // read bus
    for (int i = 0; i < len; i++) {
        if (method && !pulse_cs && addr < 0x8000U) {
            /* A15 读法沿用碳酸钙的顺序：保持 RD，逐字节用 A15 结束采样。 */
            CART_RD(0);
            cart_writeBus_ad(addr++);
            lk_delay_us(method == 2 ? 4 : 1);
            *buf++ = cart_readBus_a();
            GPIOB->BSRR = ad15_Pin;
            continue;
        }
        cart_writeBus_ad(addr);
        if (pulse_cs) {
            TIMING_DELAY();
            CART_CS1(0);
        }

        VOLATILE_32(rd_GPIO_Port->BSRR) = rd_Pin << 16;  // rd=0 126ns
        TIMING_DELAY();                                  // address to dq 105ns, oe to dq 25ns

        *buf = cart_readBus_a();
        TIMING_DELAY();

        VOLATILE_32(rd_GPIO_Port->BSRR) = rd_Pin;  // rd=1 126ns
        if (pulse_cs) CART_CS1(1);
        TIMING_DELAY();

        addr++;
        buf++;
    }

    if (method && !pulse_cs) CART_RD(1);
    // release bus
    VOLATILE_32(cs1_GPIO_Port->BSRR) = cs1_Pin;  // cs1=1 126ns
    TIMING_DELAY();
    cart_setDirection_ad(0);
}

void cart_gbcRead(uint16_t addr, uint8_t *buf, uint16_t len)
{
    gbc_read(addr, buf, len, 0, false);
}

void cart_gbcRead_jr(uint16_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t method = fgx_state.byte[11];
    bool pulse_cs = fgx_state.byte[8] != 0;
    if (method || pulse_cs) gbc_read(addr, buf, len, method, pulse_cs);
    else cart_gbcRead(addr, buf, len);
}

static inline __attribute__((always_inline)) void gbc_write(uint16_t addr, const uint8_t *buf, uint16_t len, bool pulse_cs)
{
    cart_setDirection_a(1);
    cart_setDirection_ad(1);

    if (!pulse_cs) CART_CS1(0);
    TIMING_DELAY();

    // write bus
    for (int i = 0; i < len; i++) {
        cart_writeBus_ad(addr);
        cart_writeBus_a(*buf);
        if (pulse_cs) {
            TIMING_DELAY();
            CART_CS1(0);
        }

        VOLATILE_32(wr_GPIO_Port->BSRR) = wr_Pin << 16;
        TIMING_DELAY();  // address hold 70ns, data setup 20ns, write cycle 105ns

        VOLATILE_32(wr_GPIO_Port->BSRR) = wr_Pin;
        if (pulse_cs) CART_CS1(1);
        TIMING_DELAY();

        addr++;
        buf++;
    }

    // release bus
    VOLATILE_32(cs1_GPIO_Port->BSRR) = cs1_Pin;
    TIMING_DELAY();

    cart_setDirection_a(0);
    cart_setDirection_ad(0);
}

void cart_gbcWrite(uint16_t addr, const uint8_t *buf, uint16_t len)
{
    gbc_write(addr, buf, len, false);
}

void cart_gbcWrite_jr(uint16_t addr, const uint8_t *buf, uint16_t len)
{
    if (fgx_state.byte[9]) gbc_write(addr, buf, len, true);
    else cart_gbcWrite(addr, buf, len);
}
