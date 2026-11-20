/* SPDX-License-Identifier: GPL-3.0-only
 * 碳酸钙 RC39 → STM32 硬件适配层，供保留来源的 LK 特殊算法调用。 */
#include "lk_special_hal.h"
uint8_t special_failed;
static uint8_t byte_buffer;

void lk_delay_us(uint16_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles && !flashgbx_cancelled()) {}
}
void lk_conn_send_u16(uint16_t value) {
    if(!fgx_send_word(value)) special_failed=1;
}
void lk_dmg_cart_write_byte(uint32_t address,uint16_t value) {
    if(_timeout_check()) return;
    byte_buffer=(uint8_t)value;
    cart_gbcWrite_jr(address,&byte_buffer,1);
}
uint8_t lk_dmg_cart_read_byte(uint16_t address) {
    if(_timeout_check()) return 0xff;
    cart_gbcRead_jr(address,&byte_buffer,1); return byte_buffer;
}
void fgx_dmg_flash_write(uint16_t address,uint8_t value,uint8_t pin) {
    if(flashgbx_cancelled()) return;
    if (pin == 2) { special_failed = 1; return; }  /* AUDIO 未接线。 */
    if (pin <= 1) { cart_gbcWrite_jr(address, &value, 1); return; }
    cart_setDirection_ad(1);
    cart_setDirection_a(1);
    cart_writeBus_ad(address);
    cart_writeBus_a(value);
    TIMING_DELAY();
    CART_CS2(0);
    CART_WR(0);
    lk_delay_us(1);
    CART_WR(1);
    CART_CS2(1);
    cart_setDirection_a(0);
    cart_setDirection_ad(0);
}
void lk_dmg_flash_write_byte(uint32_t address,uint16_t value) {
    if(!_timeout_check()) fgx_dmg_flash_write(address,value,fgx_state.byte[4]);
}
void lk_agb_cart_write_short(uint32_t address,uint16_t value) {
    if(_timeout_check()) return;
    cart_romWrite(address,&value,1);
}
uint16_t lk_agb_cart_read_short(uint32_t address) {
    if(_timeout_check()) return 0xffff;
    uint16_t value;
    cart_romRead(address,&value,1); return value;
}
static void verify_value(uint32_t address,uint16_t value,uint8_t status,uint8_t agb) {
    uint16_t left=20000,readback=0;
    if(_timeout_check()) return;
    do {
        if(flashgbx_cancelled()) { special_failed=1; return; }
        readback=agb?lk_agb_cart_read_short(address):lk_dmg_cart_read_byte(address);
        if(status ? ((readback&fgx_state.word[5])==fgx_state.word[6]) : readback==value) {
            /* E201264使用专用方法10，但状态位仍须按Intel检查错误，不能仅凭就绪位报成功。 */
            if(status==1 && (fgx_state.byte[2]==2 || fgx_state.byte[3]==10) && (readback & ((fgx_state.word[5]&0xff00)?0x3a3a:0x003a))) {
                fgx_state.word[3]=readback; special_failed=1;
            }
            return;
        }
    } while(--left);
    fgx_state.word[3]=readback; special_failed=1;
}
void lk_dmg_verify_data(uint32_t address,uint16_t value) { verify_value(address,(uint8_t)value,0,0); }
void lk_agb_verify_data(uint32_t address,uint16_t value) { verify_value(address,value,0,1); }
void lk_dmg_verify_status_register(uint32_t address) { verify_value(address,0,1,0); }
void lk_agb_verify_status_register(uint32_t address) { verify_value(address,0,1,1); }
/* E8提交前读取XSR，只检查就绪位；提交后才检查完整错误状态。 */
void lk_agb_verify_ready_register(uint32_t address) { verify_value(address,0,2,1); }
void lk_dmg_change_bank(uint16_t bank) {
    uint8_t i;
    if(!fgx_state.bank_count) lk_dmg_cart_write_byte(0x2000,bank);
    else for(i=0;i<fgx_state.bank_count;i++) {
        if(fgx_state.bank_mode[i]==0) lk_dmg_cart_write_byte(fgx_state.bank_address[i],bank);
        else lk_dmg_cart_write_byte(bank,fgx_state.bank_address[i]);
    }
}
