#ifndef LK_SPECIAL_API_H
#define LK_SPECIAL_API_H
#include "lk_special_hal.h"
void lk_dmg_mbc6_mmsa_write_flash_bytes(void);
void lk_dmg_mbc7_write(u8 value);
void lk_dmg_mbc7_reinit(void);
void lk_dmg_mbc7_set_write_access(u8 enable);
void lk_dmg_mbc7_set_cmd(u16 address, u8 mode);
void lk_dmg_mbc7_read_eeprom(void);
void lk_dmg_mbc7_write_eeprom(void);
void lk_dmg_eeprom_write_bytes(u16 transfer_size, u16 buffer_offset, u32 sector_address);
void lk_dmg_flash_mbc6(u16 buffer_offset);
void lk_dmg_flash_E201264(u16 buffer_offset);
void lk_dmg_flash_bung_16m(u16 buffer_offset);
void lk_dmg_flash_datel_orbitv2(void);
void lk_dmg_flash_mmsa(u16 buffer_offset);
void lk_dmg_mmsa_flash_command(u16 addr, u8 data);
void lk_dmg_mmsa_access_mapper(void);
void lk_dmg_mmsa_access_rom(void);
void lk_dmg_mmsa_access_mbc(bool enable);
void lk_dmg_mmsa_disable_flash_write_protect(void);
void lk_dmg_mmsa_map_full(void);
void lk_agb_flash2advance_buffered(u16 transfer_size, u16 buffer_offset, u32 sector_address);
void lk_agb_bootup_sequence(void);
void lk_agb_flash_fujitsu_fastmode(void);
void lk_agb_flash_paged(u16 buffer_offset);
void lk_agb_flash_gbamp(void);
#endif
