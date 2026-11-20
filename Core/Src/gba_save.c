#include "main.h"
#include "cart_adapter.h"
#include "gba_save.h"
#include "lk_special_hal.h"

/* EEPROM 的 /WR 是发送时钟，/RD 是接收时钟，AD0 是串行数据。
   地址按 8 字节块计数；4Kbit/64Kbit 分别传输 6/14 位地址。 */
static void ee_bit(uint8_t value)
{
    if (flashgbx_cancelled()) return;
    GPIOB->BSRR = value ? GPIO_PIN_0 : (uint32_t)GPIO_PIN_0 << 16;
    lk_delay_us(1); CART_WR(0); lk_delay_us(1); CART_WR(1); lk_delay_us(1);
}
static uint8_t ee_sample(void)
{
    uint8_t value;
    CART_RD(0); lk_delay_us(1); value = GPIOB->IDR & GPIO_PIN_0; CART_RD(1); lk_delay_us(1);
    return value;
}
static void ee_address(uint16_t block, uint8_t type, uint8_t reading)
{
    uint8_t bits = type == 1 ? 6 : 14;
    CART_CS1(1); CART_CS2(1); CART_RD(1); CART_WR(1);
    cart_setDirection_a(1); cart_setDirection_ad(1);
    cart_writeBus_a(0xff); cart_writeBus_ad(0xffff);
    lk_delay_us(1); CART_CS1(0);
    ee_bit(1); ee_bit(reading);
    while (bits) { --bits; ee_bit((block >> bits) & 1); }
}
static void ee_release(void)
{
    CART_CS1(1); CART_RD(1); CART_WR(1);
    cart_setDirection_ad(0); cart_setDirection_a(0);
}
void gba_eeprom_read(uint16_t block, uint8_t *data, uint8_t type)
{
    uint8_t i, bit_index, value;
    ee_address(block, type, 1); ee_bit(0); CART_CS1(1);
    cart_setDirection_ad(0); lk_delay_us(1); CART_CS1(0);
    for (i = 0; i < 4; ++i) ee_sample();
    for (i = 0; i < 8; ++i) {
        value = 0;
        for (bit_index = 0; bit_index < 8; ++bit_index) value = (value << 1) | ee_sample();
        data[i] = value;
    }
    ee_release();
}
bool gba_eeprom_write(uint16_t block, uint8_t *data, uint8_t type)
{
    uint8_t i, bit_index;
    uint16_t timeout = 20000;
    ee_address(block, type, 0);
    for (i = 0; i < 8; ++i)
        for (bit_index = 8; bit_index; --bit_index) ee_bit((data[i] >> (bit_index-1)) & 1);
    ee_bit(0); CART_CS1(1); cart_setDirection_ad(0); lk_delay_us(1); CART_CS1(0);
    while (timeout) { if (flashgbx_cancelled()) { timeout=0; break; } if (ee_sample()) break; --timeout; }
    ee_release();
    return timeout != 0;
}

/* RTC 寄存器地址从 GBA 字节地址换算为 16 位字地址。 */
static uint16_t gpio_word;
static void gpio_write(uint16_t address, uint8_t value)
{
    gpio_word = value;
    cart_romWrite(address >> 1, &gpio_word, 1);
}
static void rtc_begin(uint8_t command)
{
    uint8_t i, value;
    gpio_write(0xc8, 1); gpio_write(0xc4, 1); gpio_write(0xc4, 5); gpio_write(0xc6, 7);
    for (i = 8; i; --i) {
        value = 4 | (((command >> (i-1)) & 1) << 1);
        gpio_write(0xc4, value); gpio_write(0xc4, value); gpio_write(0xc4, value | 1);
    }
    gpio_write(0xc6, 5);
}
static uint8_t rtc_byte(void)
{
    uint8_t i, value = 0;
    for (i = 0; i < 8; ++i) {
        gpio_write(0xc4, 4); gpio_write(0xc4, 4); gpio_write(0xc4, 5);
        cart_romRead(0x62, &gpio_word, 1);
        value = (value >> 1) | ((gpio_word & 2) << 6);
    }
    return value;
}
static void rtc_end(void) { gpio_write(0xc4, 1); gpio_write(0xc8, 0); }
void gba_rtc_read(uint8_t *data)
{
    uint8_t i;
    rtc_begin(0x63); data[0] = rtc_byte(); rtc_end();
    rtc_begin(0x65);
    for (i = 0; i < 7; ++i) data[i+1] = rtc_byte();
    rtc_end();
}
