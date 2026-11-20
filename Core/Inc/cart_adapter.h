#ifndef __CART_ADAPTER_H_
#define __CART_ADAPTER_H_

#include <stdbool.h>

// Memory barrier macro to prevent compiler optimization issues
#define MEMORY_BARRIER() __asm volatile("" ::: "memory")

// Function attribute to prevent aggressive optimization
#define NO_OPTIMIZE __attribute__((optimize("O0")))
#define NO_INLINE __attribute__((noinline))

#define VOLATILE_CAST(type, addr) (*(volatile type *)(addr))
#define VOLATILE_32(var) (*(volatile uint32_t *)(&var))
#define VOLATILE_16(var) (*(volatile uint16_t *)(&var))
#define VOLATILE_8(var) (*(volatile uint8_t *)(&var))

// Short delay macro for timing critical operations
// Increased delay for release builds to ensure timing requirements
#ifdef NDEBUG
    #define TIMING_DELAY()                                                                         \
        do {                                                                                       \
            __asm volatile("nop");                                                                 \
            __asm volatile("nop");                                                                 \
            __asm volatile("nop");                                                                 \
            __asm volatile("nop");                                                                 \
            __asm volatile("nop");                                                                 \
            __asm volatile("nop");                                                                 \
        } while (0)
#else
    #define TIMING_DELAY()                                                                         \
        do {                                                                                       \
            __asm volatile("nop");                                                                 \
            __asm volatile("nop");                                                                 \
            __asm volatile("nop");                                                                 \
        } while (0)
#endif

#define CART_PIN(port, pin, high) ((port)->BSRR = (high) ? (pin) : (uint32_t)(pin) << 16)
#define CART_CS1(high) CART_PIN(cs1_GPIO_Port, cs1_Pin, high)
#define CART_CS2(high) CART_PIN(cs2_GPIO_Port, cs2_Pin, high)
#define CART_RD(high) CART_PIN(rd_GPIO_Port, rd_Pin, high)
#define CART_WR(high) CART_PIN(wr_GPIO_Port, wr_Pin, high)

void cart_setDirection_ad(uint8_t dir);
void cart_setDirection_a(uint8_t dir);
void cart_writeBus_ad(uint16_t value);
void cart_writeBus_a(uint8_t value);
void cart_romRead_begin(uint32_t addr);
void cart_romRead_continue(uint16_t *buf, uint16_t len);
bool cart_romRead_verify(const uint8_t *expected, uint16_t *buf, uint16_t words);
uint32_t cart_romRead_crc32(uint32_t crc, const uint32_t *table, uint16_t words);
void cart_romRead_end(void);
uint16_t cart_romPoll_read(uint16_t address);
void cart_romRead(uint32_t addr, uint16_t *buf, uint16_t len);
void cart_romWrite(uint32_t addr, const uint16_t *buf, uint16_t len);
static inline void cart_romWrite_setup(void)
{
    cart_setDirection_a(1);
    cart_setDirection_ad(1);
}
void cart_romWrite_command(uint32_t addr, uint16_t value);
void cart_romWrite_bytes(uint32_t addr, const uint8_t *buf, uint16_t words);
static inline void cart_romWrite_finish(void)
{
    cart_setDirection_a(0);
    cart_setDirection_ad(0);
}
void cart_ramRead(uint16_t addr, uint8_t *buf, uint16_t len);
void cart_ramWrite(uint16_t addr, const uint8_t *buf, uint16_t len);

void cart_gbcRead(uint16_t addr, uint8_t *buf, uint16_t len);
void cart_gbcWrite(uint16_t addr, const uint8_t *buf, uint16_t len);
void cart_gbcRead_jr(uint16_t addr, uint8_t *buf, uint16_t len);
void cart_gbcWrite_jr(uint16_t addr, const uint8_t *buf, uint16_t len);

#endif
