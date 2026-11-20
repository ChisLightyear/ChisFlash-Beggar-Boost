#ifndef LK_SPECIAL_HAL_H
#define LK_SPECIAL_HAL_H
#include "fgx_state.h"
#include "cart_adapter.h"
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
extern uint8_t cmdBuf[5500];
extern uint8_t special_failed;
void lk_delay_us(uint16_t us);
void lk_dmg_cart_write_byte(uint32_t address,uint16_t value);
void lk_dmg_flash_write_byte(uint32_t address,uint16_t value);
uint8_t lk_dmg_cart_read_byte(uint16_t address);
void lk_agb_cart_write_short(uint32_t address,uint16_t value);
uint16_t lk_agb_cart_read_short(uint32_t address);
void lk_dmg_verify_data(uint32_t address,uint16_t value);
void lk_agb_verify_data(uint32_t address,uint16_t value);
void lk_dmg_verify_status_register(uint32_t address);
void lk_agb_verify_status_register(uint32_t address);
void lk_agb_verify_ready_register(uint32_t address);
void lk_dmg_change_bank(uint16_t bank);
void lk_conn_send_u16(uint16_t value);
void fgx_dmg_flash_write(uint16_t address,uint8_t value,uint8_t pin);
#define _delay_us(n) lk_delay_us(n)
#define _delay_ms(n) lk_delay_us((n) * 1000U)
#define _delay_200ns() TIMING_DELAY()
#define _delay_400ns() lk_delay_us(1)
#define _delay_500ns() lk_delay_us(1)
#define _timeout_check() (special_failed || flashgbx_cancelled())
#define RAW_DMG_DATA_DIR_OUT() cart_setDirection_a(1)
#define RAW_DMG_DATA_DIR_IN() cart_setDirection_a(0)
#define RAW_DMG_DATA_SET(n) cart_writeBus_a(n)
#define RAW_AGB_ADDR_DIR_OUT() do { cart_setDirection_ad(1); cart_setDirection_a(1); } while (0)
#define RAW_AGB_DATA_DIR_OUT() cart_setDirection_ad(1)
#define RAW_AGB_ADDR_SET(n) do { cart_writeBus_ad(n); cart_writeBus_a((uint32_t)(n) >> 16); } while (0)
#define RAW_AGB_DATA_SET(n) cart_writeBus_ad(n)
#define PIN_CS_L() do { if (!_timeout_check()) CART_CS1(0); } while (0)
#define PIN_CS_H() CART_CS1(1)
#define PIN_WR_L() do { if (!_timeout_check()) CART_WR(0); } while (0)
#define PIN_WR_H() CART_WR(1)
#define PIN_RD_L() CART_RD(0)
#define PIN_RD_H() CART_RD(1)
#endif
