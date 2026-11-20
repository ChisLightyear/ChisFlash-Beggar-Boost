#include <string.h>

#include "main.h"
#include "usbd_cdc_if.h"
#include "cart_adapter.h"
#include "flashgbx.h"
#include "fgx_state.h"
#include "gba_save.h"
#include "lk_special_api.h"
#include "crc32_table.h"

/* RC39 FlashGBX 协议移植。控制字段为大端，卡带数据保持原始字节顺序。 */
#define FGX_LIMIT 4096
#define FGX_READ_LIMIT 8192
#define FGX_CARD_POWER 1

extern USBD_HandleTypeDef hUsbDeviceFS;
extern volatile uint16_t cmdBuf_p;
extern volatile uint8_t busy;
extern uint8_t cmdBuf[5500], responBuf[5500];

FgxState fgx_state;
#define state fgx_state
#define ADDRESS state.wide[0]
#define TRANSFER state.word[0]
#define BUFFER state.word[1]
#define MODE state.byte[0]

static volatile uint32_t session_id;
static uint32_t transaction_id;
static volatile bool active;
static bool transport_fault;
static uint8_t pending, pending_type;
static uint16_t memory3d_offset;
static bool memory3d_active;
static uint8_t reply[64], bus_word[2];
static const uint8_t probe_reply[] = "\x01" "ChisFlash Beggar FW C15 RC39-STM32\r\n";

bool flashgbx_cancelled(void)
{
    return transaction_id != session_id;
}

static void release_bus(void)
{
    CART_RD(1);
    CART_WR(1);
    CART_CS1(1);
    CART_CS2(1);
    cart_setDirection_ad(0);
    cart_setDirection_a(0);
}

void flashgbx_reset(void)
{
    ++session_id;
    if (active) {
        active = false;
        cmdBuf_p = 0;
    }
}

static void state_defaults(void)
{
    memset(&state, 0, sizeof(state));
    state.byte[14] = 1;
    state.word[5] = 0x80;
    state.word[6] = 0x80;
    state.selected_voltage = 1;
    pending = 0;
    transport_fault = false;
    memory3d_active = false;
    release_bus();

    /* 使用 Cortex-M3 周期计数器提供特殊卡带协议所需的微秒延时。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void consume(uint16_t count)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!flashgbx_cancelled()) {
        if (transport_fault) {
            cmdBuf_p = 0;
        } else {
            memmove(cmdBuf, cmdBuf + count, cmdBuf_p - count);
            cmdBuf_p -= count;
        }
    }
    __set_PRIMASK(mask);
    if (flashgbx_cancelled()) release_bus();
}

static bool send_packet(uint8_t *data, uint16_t size)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    USBD_CDC_HandleTypeDef *cdc = hUsbDeviceFS.pClassData;
    bool started = !flashgbx_cancelled() && cdc && CDC_Transmit_FS(data, size) == USBD_OK;
    __set_PRIMASK(mask);
    if (!started) return false;

    /* CDC 在发送完成前仍引用源缓冲区，并为 64 字节整倍数添加 ZLP。 */
    while (!flashgbx_cancelled()) {
        __disable_irq();
        cdc = hUsbDeviceFS.pClassData;
        bool sending = cdc && cdc->TxState;
        __set_PRIMASK(mask);
        if (!sending) return !flashgbx_cancelled();
    }
    return false;
}

static void ack(uint8_t value)
{
    reply[0] = value;
    send_packet(reply, 1);
}

bool fgx_send_word(uint16_t value)
{
    reply[0] = value >> 8;
    reply[1] = value;
    return send_packet(reply, 2);
}

static uint32_t be32(uint16_t p) {
    return ((uint32_t)cmdBuf[p]<<24)|((uint32_t)cmdBuf[p+1]<<16)|((uint32_t)cmdBuf[p+2]<<8)|cmdBuf[p+3];
}
static uint16_t be16(uint16_t p) { return ((uint16_t)cmdBuf[p]<<8)|cmdBuf[p+1]; }
/* 配置中的 AMD 命令已按卡槽位序编码；只转换固件临时生成的控制字段，不转换 ROM 数据。 */
static uint8_t amd_wire_byte(uint8_t value,uint8_t unlock) {
    if((unlock&3)==1 && ((value^(value>>1))&1)) value^=3;
    if((unlock&0xc0)==0x40 && ((value^(value>>1))&0x40)) value^=0xc0;
    return value;
}
static bool amd_commands_valid(void) {
    uint8_t unlock=(uint8_t)be16(8);
    return unlock==amd_wire_byte(0xaa,unlock) &&
        (uint8_t)be16(14)==amd_wire_byte(0x55,unlock) &&
        (uint8_t)be16(20)==amd_wire_byte(cmdBuf[2]==1?0xa0:0x25,unlock) &&
        (cmdBuf[2]!=2 || (uint8_t)be16(38)==amd_wire_byte(0x29,unlock));
}
static void put32(uint32_t v) { reply[0]=v>>24; reply[1]=v>>16; reply[2]=v>>8; reply[3]=v; }
static uint16_t expected(void) {
    if(pending==0xaf) return sizeof(state);
    if(pending) return TRANSFER;
    if(!cmdBuf_p) return 1;
    switch(cmdBuf[0]) {
    case 0x55: case 0x4c: case 0xfe: case 0xc5: case 0xc6: case 0xc7: return 2;
    case 0xa6: return 10;
    case 0xad: case 0xb2: case 0xd1: case 0xf5: return 6;
    case 0xc2: case 0xd2: return 7;
    case 0xa9: case 0xd5: return 5;
    case 0xa7: return 40;
    case 0xb8: return cmdBuf_p<2?2:2+(uint16_t)cmdBuf[1]*5;
    case 0xd4: return cmdBuf_p<3?3:3+(uint16_t)cmdBuf[2]*6;
    default: return 1;
    }
}
static void query_info(void)
{
    static const char name[] = "ChisFlash Beggar";
    const uint32_t timestamp = BUILD_TIMESTAMP;
    reply[0] = 8;
    reply[1] = 'C';
    reply[2] = 0;
    reply[3] = 15;
    reply[4] = 3;
    reply[5] = timestamp >> 24;
    reply[6] = (uint8_t)(timestamp >> 16);
    reply[7] = (uint8_t)(timestamp >> 8);
    reply[8] = (uint8_t)timestamp;
    reply[9] = sizeof(name) - 1;
    memcpy(reply + 10, name, sizeof(name) - 1);
    reply[sizeof(name) + 9] = 0;  /* 固定供电，无模式与插卡开关。 */
    reply[sizeof(name) + 10] = 0;
    send_packet(reply, sizeof(name) + 11);
}

static uint32_t get_variable(uint8_t size,uint32_t key) {
    if(size==4 && key<2) return state.wide[(uint8_t)key];
    if(size==2 && key<7) return state.word[(uint8_t)key];
    if(size==1 && key==13) return FGX_CARD_POWER;
    if(size==1 && key<18) return state.byte[(uint8_t)key];
    return 0;
}
static bool set_variable(uint8_t size,uint32_t key,uint32_t v) {
    if(size==4 && key<2) {
        if(key==1 && v>32767000UL) return 0;
        state.wide[(uint8_t)key]=v;
        return 1;
    }
    if(size==2 && key<7) {
        if(v>65535UL || (key==0 && v>FGX_READ_LIMIT)) return 0;
        state.word[(uint8_t)key]=v;
        if(key==2) state.word[4]=v&255;
        return 1;
    }
    if(size==1 && key<18 && v<=255) {
        if(key==0 && v>2) return 0;
        if(key==3 && v>12) return 0;
        if(key==13 || ((key==15 || key==16) && v>1)) return 0;
        if(key==14 && v>3) return 0;
        if((key==11 || key==12) && v>2) return 0;
        if(key==4 && (v>3 || v==2)) return 0;
        if((key==5 || key==6 || key==10 || key==17) && v>1) return 0;
        if((key==0 || key==2 || key==3 || key==4) && state.byte[(uint8_t)key]!=v) state.configured=0;
        state.byte[(uint8_t)key]=v;
        return 1;
    }
    return 0;
}
/* 低 16 位字地址跨界时重新锁存，和碳酸钙的 GBA 快路径一致。 */
static uint16_t gba_read_span(uint32_t address, uint16_t bytes)
{
    uint32_t window_bytes = (0x10000UL - (uint16_t)address) * 2U;
    return bytes > window_bytes ? (uint16_t)window_bytes : bytes;
}

static void read_chunk(uint8_t kind, uint32_t address, uint16_t count)
{
    if (kind == 0xb1) cart_gbcRead_jr(address, responBuf, count);
    else if (kind == 0xc3) cart_ramRead(address, responBuf, count);
    else {
        uint16_t span = gba_read_span(address, count);
        cart_romRead(address, (uint16_t *)responBuf, (span + 1U) / 2U);
        if (span != count) {
            cart_romRead(address + span / 2U, (uint16_t *)(responBuf + span),
                         (count - span + 1U) / 2U);
        }
    }
}

static void read_stream(uint8_t kind)
{
    uint16_t left = TRANSFER;
    if (!left || left > FGX_READ_LIMIT || (kind == 0xb1 ? MODE != 1 : MODE != 2)) {
        ack(2);
        return;
    }
    /* GBA 高位属于主机逻辑地址，卡带总线仍按 24 位字地址访问。 */
    uint32_t address = kind == 0xc1 ? ADDRESS & 0xffffffUL : ADDRESS;
    uint32_t limit = kind == 0xc1 ? 0x1000000UL : 0x10000UL;
    if (address >= limit || (kind == 0xc1 ? (left + 1U) / 2U : left) > limit - address) {
        ack(2);
        return;
    }
    while (left && !flashgbx_cancelled()) {
        uint16_t count = left > FGX_LIMIT ? FGX_LIMIT : left;
        read_chunk(kind, ADDRESS, count);
        if (!send_packet(responBuf, count)) return;
        ADDRESS += kind == 0xc1 ? (count + 1U) / 2U : count;
        left -= count;
    }
}

static void write_unit(uint32_t address, uint16_t value, bool rom)
{
    if (flashgbx_cancelled()) return;
    uint8_t byte_value = value;
    if (MODE == 1 && rom) fgx_dmg_flash_write(address, byte_value, state.byte[4]);
    else if (MODE == 1) cart_gbcWrite_jr(address, &byte_value, 1);
    else if (rom) cart_romWrite(address, &value, 1);
    else cart_ramWrite(address, &byte_value, 1);
}

static uint16_t read_unit(uint32_t address)
{
    if (MODE == 1) {
        cart_gbcRead_jr(address, bus_word, 1);
        return bus_word[0];
    }
    uint16_t value;
    cart_romRead(address, &value, 1);
    return value;
}

static bool wait_program(uint32_t address,uint16_t v) {
    uint16_t attempts=20000,sample;
    while(attempts--) {
        if(flashgbx_cancelled()) return 0;
        sample=read_unit(address);
        if(MODE==1) { sample&=255; v&=255; }
        if(sample==v) return 1;
    }
    state.word[3]=sample;
    return 0;
}
static void flash_command(uint8_t index)
{
    write_unit(state.commands[index].address, state.commands[index].value, true);
}
static bool program_save(uint8_t type);
static bool fixed_method(uint8_t m) { return m==3 || m==4 || m==9 || m==10 || m==11 || m==12; }
static bool program_special(void) {
    uint8_t method=state.byte[3];
    uint16_t pos,step;
    uint32_t low=ADDRESS&0xffffUL;
    special_failed=0;
    if(!TRANSFER || TRANSFER>FGX_LIMIT) return 0;
    if(method==7) return program_save(1);
    if(method==3 || method==4 || method==9 || method==10 || method==12) {
        if(MODE!=1 || low+TRANSFER>0x10000UL || (method!=9 && TRANSFER%128)) return 0;
    } else if(MODE!=2 || (TRANSFER&1) || ADDRESS>=0x1000000UL || TRANSFER/2>0x1000000UL-ADDRESS) return 0;
    if(method==11 && ADDRESS+TRANSFER/2>0x200000UL) return 0;
    if(method==5 || method==8) {
        if(!BUFFER || BUFFER>FGX_LIMIT || TRANSFER%BUFFER || BUFFER%4) return 0;
    }
    if(method==9) lk_dmg_flash_datel_orbitv2();
    else if(method==6) lk_agb_flash_fujitsu_fastmode();
    else if(method==11) lk_agb_flash_gbamp();
    else {
        step=(method==5 || method==8)?BUFFER:128;
        for(pos=0;pos<TRANSFER && !special_failed && !flashgbx_cancelled();pos+=step) {
            switch(method) {
            case 3: lk_dmg_flash_mmsa(pos); break;
            case 4: lk_dmg_flash_mbc6(pos); break;
            case 5: lk_agb_flash2advance_buffered(step,pos,ADDRESS); break;
            case 8: lk_agb_flash_paged(pos); break;
            case 10: lk_dmg_flash_E201264(pos); break;
            case 12: lk_dmg_flash_bung_16m(pos); break;
            default: return 0;
            }
        }
    }
    if(method==3 || method==4) write_unit((ADDRESS-1)&0xffffUL,0xf0,0);
    if(method==8) write_unit(ADDRESS-1,0xff,1);
    return !special_failed && !flashgbx_cancelled();
}
static bool valid_range(uint32_t length, uint8_t rom) {
    uint32_t limit = MODE==2 && rom ? 0x1000000UL : 0x10000UL;
    uint32_t address=ADDRESS;
    if(MODE!=1 && MODE!=2) return 0;
    /* 只按 ROM 总线的 24 位字地址检查窗口边界，保留 ADDRESS 的逻辑高位和递增语义。 */
    if(MODE==2 && rom) { address&=0xffffffUL; length=(length+1)/2; }
    return address<limit && length<=limit-address;
}
/* 仅转换固件生成的控制值；上传命令与 ROM 字节均保持卡槽位序。
   E8 在 D6/D7 对调后数值不变，使用上位机提供的就绪位识别该映射。 */
static uint16_t intel_wire_value(uint16_t value) {
    if(state.commands[0].value==0xea && ((value^(value>>1))&1)) value^=3;
    if((state.word[5]&0xc0)==0x40 && ((value^(value>>1))&0x40)) value^=0xc0;
    return value;
}
static bool intel_ready(uint32_t address,uint8_t check_errors) {
    uint16_t tries=20000,value,error_mask=state.commands[0].value==0xea?0x39:0x3a;
    if(!state.word[5]) return 0;
    if(state.word[5]&0xff00) error_mask|=0x3a00;
    while(tries--) {
        if(flashgbx_cancelled()) return 0;
        value=read_unit(address); state.word[3]=value;
        /* E8 的 XSR 只使用就绪位；提交后才检查完整错误状态。 */
        if((value&state.word[5])==state.word[6]) return !check_errors || (value&error_mask)==0;
    }
    return 0;
}
static uint16_t page_remaining(uint32_t address,uint16_t buffer,uint8_t unit) {
    uint16_t mask=buffer-1,offset=(uint16_t)address;
    /* 常见 Flash 页大小为 2 的幂，低 16 位足以计算余数；其它页大小保留原算法。 */
    if(!(buffer&mask)) {
        if(unit==2) offset<<=1;
        return buffer-(offset&mask);
    }
    return buffer-(uint16_t)((address*unit)%buffer);
}
static bool verify_chunk(uint16_t offset, uint16_t count, uint16_t buffer_offset)
{
    for (uint16_t i = 0; i < count; ++i) {
        if (responBuf[buffer_offset + i] != cmdBuf[offset + i]) {
            state.word[3] = responBuf[buffer_offset + i];
            return false;
        }
    }
    return true;
}

static bool verify_gba_word(uint16_t offset)
{
    uint16_t actual, expected;
    __builtin_memcpy(&actual, responBuf, sizeof(actual));
    __builtin_memcpy(&expected, cmdBuf + offset, sizeof(expected));
    return actual == expected;
}

static bool verify_gba_buffer(uint16_t offset, uint16_t count)
{
    if (cart_romRead_verify(cmdBuf + offset, (uint16_t *)responBuf, count / 2U)) {
        return true;
    }
    return verify_chunk(offset, count, 0);
}

static bool verify_gba_page(uint32_t address, uint16_t offset, uint16_t length,
                            bool intel)
{
    /* Intel 仍逐字判错。AMD 仍以原 64 字节读取批次判错。 */
    uint16_t group = intel ? 2U : 64U;
    while (length && !flashgbx_cancelled()) {
        uint16_t span = gba_read_span(address, length);
        cart_romRead_begin(address);
        for (uint16_t left = span; left != 0;) {
            if (flashgbx_cancelled()) {
                cart_romRead_end();
                return false;
            }
            uint16_t count = left < group ? left : group;
            bool verified;
            if (intel) {
                cart_romRead_continue((uint16_t *)responBuf, 1);
                verified = verify_gba_word(offset);
            } else {
                verified = verify_gba_buffer(offset, count);
            }
            if (!verified) {
                cart_romRead_end();
                return false;
            }
            left -= count;
            offset += count;
        }
        cart_romRead_end();
        length -= span;
        address += span / 2U;
    }
    return !flashgbx_cancelled();
}

static bool program_intel(void) {
    uint16_t pos=0,n,j,v;
    uint8_t unit=MODE==2?2:1;
    uint32_t start;
    while(pos<TRANSFER) {
        if(flashgbx_cancelled()) return 0;
        start=ADDRESS;
        if(state.byte[3]==1) {
            if(state.commands[0].value==0x70 || state.commands[0].value==0xb0) {
                write_unit(start,state.commands[0].value,1);
                if(!intel_ready(start,1)) goto failed;
                write_unit(start,state.commands[1].value,1);
            } else write_unit(start,state.commands[0].value,1);
            n=unit;
        } else {
            n=page_remaining(ADDRESS,BUFFER,unit);
            if(n>TRANSFER-pos) n=TRANSFER-pos;
            /* E9/EA 使用至多 512 字缓冲；E8 保留既有上限。 */
            j=MODE==2 && state.commands[0].value!=0xe8?1024:256*unit;
            if(n>j) n=j;
            write_unit(start,intel_wire_value(0x50),1); write_unit(start,0xff,1);
            write_unit(start,state.commands[0].value,1);
            /* G18 E9/EA 在 D0 提交后才切到状态模式，不能拿提交前的阵列值判断就绪。 */
            if(state.commands[0].value==0xe8 && !intel_ready(start,0)) goto failed;
            v=(unit==2?(n>>1):n)-1;
            write_unit(start,intel_wire_value(v),1);
        }
        /* G18 的 WR 不保证递增地址：每个字都重新锁存完整地址。
           与实机 D4 逐字寻址对照一致，不沿用 AMD 卡板的连续写入假设。 */
        for(j=0;j<n;j+=unit) {
            v=cmdBuf[pos+j]; if(unit==2) v|=(uint16_t)cmdBuf[pos+j+1]<<8;
            write_unit(ADDRESS,v,1); ++ADDRESS;
        }
        if(state.byte[3]==2) write_unit(start,0xd0,1);
        if(!intel_ready(start,1)) goto failed;
        write_unit(start,0xff,1);
        if(MODE==2) {
            if (!verify_gba_page(start, pos, n, true)) goto failed;
        } else for(j=0;j<n;j+=unit) {
            if(read_unit(start+j)!=cmdBuf[pos+j]) goto failed;
        }
        pos+=n;
    }
    return 1;
failed:
    write_unit(start,intel_wire_value(0x50),1); write_unit(start,0xff,1); return 0;
}
/* AMD 单字节/字和缓冲编程；擦除指令由主机 D4 发出并轮询。
   未覆盖算法在 A7 返回错误，不能当 SRAM 写后报告编程成功。 */
static uint16_t amd_page_count(uint8_t count)
{
    uint16_t unlock = state.commands[0].value;
    uint16_t value = amd_wire_byte(count, (uint8_t)unlock);
    if (state.byte[10]) value |= (uint16_t)amd_wire_byte(count, unlock >> 8) << 8;
    return value;
}

static void write_gba_page_command(uint32_t address, uint16_t value)
{
    if (!flashgbx_cancelled()) cart_romWrite_command(address, value);
}

static bool write_gba_amd_page(uint32_t address, uint16_t pos, uint16_t bytes)
{
    /* 整页写入共用总线方向，每笔命令仍单独锁存地址并产生原片选脉冲。 */
    cart_romWrite_setup();
    write_gba_page_command(state.commands[0].address, state.commands[0].value);
    write_gba_page_command(state.commands[1].address, state.commands[1].value);
    write_gba_page_command(address, state.commands[2].value);
    write_gba_page_command(address, amd_page_count(bytes / 2U - 1U));
    if (!flashgbx_cancelled()) {
        cart_romWrite_bytes(address, cmdBuf + pos, bytes / 2U);
        write_gba_page_command(address, state.commands[5].value);
    }
    cart_romWrite_finish();
    return !flashgbx_cancelled();
}

static bool program_flash(void) {
    uint16_t pos=0,n,unit=MODE==2?2:1,j,v=0;
    uint8_t verify_count;
    uint32_t start,last=ADDRESS;
    if(!state.configured || (state.byte[2]!=1 && state.byte[2]!=2) || (MODE!=1 && MODE!=2)) return 0;
    if(!TRANSFER || TRANSFER>FGX_LIMIT || (MODE==2 && (TRANSFER&1))) return 0;
    if(!valid_range(TRANSFER,1)) return 0;
    if(state.byte[3]==2 && (BUFFER<unit || BUFFER>FGX_LIMIT || BUFFER%unit)) return 0;
    if(state.byte[2]==2) return program_intel();
    bool gba_buffered = MODE == 2 && state.byte[3] == 2;
    while(pos<TRANSFER) {
        if(flashgbx_cancelled()) return 0;
        start=ADDRESS;
        if(state.byte[3]==1) {
            if(MODE==1 && state.byte[6]) { special_failed=0; lk_dmg_change_bank(1); }
            flash_command(0); flash_command(1); flash_command(2);
            if(MODE==1 && state.byte[6]) lk_dmg_change_bank(state.word[4]);
            n=unit;
        }
        else {
            n=page_remaining(ADDRESS,BUFFER,unit);
            if(n>TRANSFER-pos) n=TRANSFER-pos;
            if(n>256*unit) n=256*unit;
            if (!gba_buffered) {
                flash_command(0); flash_command(1);
                write_unit(start,state.commands[2].value,1);
                write_unit(start,amd_page_count(n - 1U),1);
            }
        }
        if (gba_buffered) {
            if (!write_gba_amd_page(start, pos, n)) return false;
            ADDRESS += n / 2;
            last = ADDRESS - 1;
            v = cmdBuf[pos+n-2] | ((uint16_t)cmdBuf[pos+n-1] << 8);
        } else if (MODE == 1 && state.byte[3] == 2 && state.byte[4] <= 1 && !state.byte[6] &&
                   ADDRESS < 0x8000UL && n <= 0x8000UL - ADDRESS) {
            if (flashgbx_cancelled()) return false;
            cart_gbcWrite_jr(ADDRESS, cmdBuf + pos, n);
            ADDRESS += n;
            last = ADDRESS - 1;
            v = cmdBuf[pos+n-1];
        } else {
            for(j=0;j<n;j+=unit) {
                v=cmdBuf[pos+j]; if(unit==2) v|=(uint16_t)cmdBuf[pos+j+1]<<8;
                last=ADDRESS; write_unit(ADDRESS,v,1); ++ADDRESS;
            }
        }
        if(state.byte[3]==2 && !gba_buffered) write_unit(start,state.commands[5].value,1);
        if(MODE==1 && state.byte[5]) {
            lk_delay_us(10); CART_CS2(0); lk_delay_us(3); CART_CS2(1);
            if(state.word[2]) { special_failed=0; lk_dmg_change_bank(state.word[4]); }
        }
        if(!wait_program(last,v)) { write_unit(0,0xf0,1); return 0; }
        /* 末字为 FF 时，无响应或拒绝编程的芯片也可能通过末字轮询。
           按页回读所有数据，确认真实写入后才发送成功 ACK。 */
        if (MODE == 2) {
            bool verified;
            if (n <= 64 && n == gba_read_span(start, n)) {
                if (flashgbx_cancelled()) return false;
                cart_romRead_begin(start);
                verified = verify_gba_buffer(pos, n);
                cart_romRead_end();
            } else {
                verified = verify_gba_page(start, pos, n, false);
            }
            if (!verified) { write_unit(0,0xf0,1); return 0; }
        } else {
            for(j=0;j<n;j+=verify_count) {
                if(flashgbx_cancelled()) return 0;
                verify_count=n-j>64?64:(uint8_t)(n-j);
                read_chunk(0xb1,start+j,verify_count);
                if(!verify_chunk(pos+j,verify_count,0)) { write_unit(0,0xf0,1); return 0; }
            }
        }
        pos+=n;
    }
    return 1;
}
static uint32_t crc_block(uint32_t crc, uint16_t count)
{
    for (uint16_t i = 0; i < count; ++i) {
        crc = (crc >> 8) ^ crc32_table[(uint8_t)(crc ^ responBuf[i])];
    }
    return crc;
}

static void crc_rom(uint32_t left)
{
    uint32_t address = ADDRESS;
    uint32_t crc = 0xffffffffUL;
    if (!valid_range(left, 1) || left > 0x20000UL) { ack(2); return; }
    if (MODE == 1) {
        while (left && !flashgbx_cancelled()) {
            uint16_t count = left > FGX_LIMIT ? FGX_LIMIT : left;
            if (address >= 0x8000UL || count > 0x8000UL - address) {
                count = left > 64 ? 64 : left;
            }
            read_chunk(0xb1, address, count);
            crc = crc_block(crc, count);
            address += count;
            left -= count;
        }
    } else {
        while (left && !flashgbx_cancelled()) {
            uint16_t bytes = left > FGX_LIMIT ? FGX_LIMIT : left;
            uint16_t span = gba_read_span(address, bytes);
            cart_romRead_begin(address);
            for (uint16_t remaining = span; remaining && !flashgbx_cancelled();) {
                uint16_t count = remaining > 64 ? 64 : remaining;
                crc = cart_romRead_crc32(crc, crc32_table, count / 2U);
                if (count & 1U) {
                    cart_romRead_continue((uint16_t *)responBuf, 1);
                    crc = crc_block(crc, 1);
                }
                remaining -= count;
            }
            cart_romRead_end();
            address += (span + 1U) / 2U;
            left -= span;
        }
    }
    if (flashgbx_cancelled()) return;
    put32(~crc);
    send_packet(reply, 4);
}
static bool eeprom_range(uint8_t type) {
    uint16_t blocks=type==1?64:1024;
    return MODE==2 && (type==1 || type==2) && TRANSFER && !(TRANSFER&7) &&
           ADDRESS<blocks && TRANSFER/8<=blocks-ADDRESS;
}
static void read_eeprom(uint8_t type) {
    uint16_t pos;
    uint8_t used=0;
    if(!eeprom_range(type)) { ack(2); return; }
    for(pos=0;pos<TRANSFER;pos+=8) {
        if(flashgbx_cancelled()) return;
        gba_eeprom_read((uint16_t)ADDRESS,responBuf+used,type);
        ++ADDRESS; used+=8;
        if(used==64 || pos+8==TRANSFER) { if(!send_packet(responBuf,used)) return; used=0; }
    }
}
static bool write_eeprom(uint8_t type) {
    uint16_t pos;
    uint8_t i;
    if(!eeprom_range(type)) return 0;
    for(pos=0;pos<TRANSFER;pos+=8) {
        if(flashgbx_cancelled() || !gba_eeprom_write((uint16_t)ADDRESS,cmdBuf+pos,type)) return 0;
        gba_eeprom_read((uint16_t)ADDRESS,responBuf,type);
        for(i=0;i<8;i++) if(responBuf[i]!=cmdBuf[pos+i]) return 0;
        ++ADDRESS;
    }
    return 1;
}
static bool program_save(uint8_t type) {
    uint16_t pos,n,j,tries;
    uint32_t start;
    if(MODE!=2 || (type!=1 && type!=2) || (type==2 && (TRANSFER%128))) return 0;
    start=type==2?ADDRESS*128:ADDRESS;
    if(start>=65536UL || TRANSFER>65536UL-start) return 0;
    for(pos=0;pos<TRANSFER;pos+=n) {
        if(flashgbx_cancelled()) return 0;
        n=type==1?1:128;
        write_unit(0x5555,0xaa,0); write_unit(0x2aaa,0x55,0); write_unit(0x5555,0xa0,0);
        cart_ramWrite(start+pos,cmdBuf+pos,n);
        tries=20000;
        do {
            if(flashgbx_cancelled()) return 0;
            cart_ramRead(start+pos+n-1,bus_word,1);
            if(bus_word[0]==cmdBuf[pos+n-1]) break;
        } while(--tries);
        if(!tries) { write_unit(0,0xf0,0); return 0; }
        for(j=0;j<n;j++) {
            cart_ramRead(start+pos+j,bus_word,1);
            if(bus_word[0]!=cmdBuf[pos+j]) return 0;
        }
    }
    ADDRESS+=type==2?TRANSFER/128:TRANSFER;
    return 1;
}
static bool dmg_page_write(void) {
    uint16_t pos,n;
    special_failed=0;
    if(MODE!=1 || !state.configured || !TRANSFER || !BUFFER || BUFFER>256 || !valid_range(TRANSFER,0)) return 0;
    for(pos=0;pos<TRANSFER && !special_failed && !flashgbx_cancelled();pos+=n) {
        n=BUFFER-(uint16_t)(ADDRESS%BUFFER);
        if(n>TRANSFER-pos) n=TRANSFER-pos;
        lk_dmg_eeprom_write_bytes(n,pos,ADDRESS);
        lk_dmg_verify_data(ADDRESS-1,cmdBuf[pos+n-1]);
    }
    return !special_failed && !flashgbx_cancelled();
}
static void read_3d(void) {
    uint16_t left,n;
    uint32_t bytes_address;
    if(MODE!=2 || !TRANSFER || TRANSFER>BUFFER || !BUFFER || BUFFER>4096 || BUFFER%512 || BUFFER%TRANSFER || (TRANSFER&1) || ADDRESS>0x7fffffffUL-BUFFER/2) { ack(2); return; }
    if(memory3d_active && memory3d_offset==BUFFER) {
        CART_CS1(1); ADDRESS+=BUFFER/2; memory3d_active=0;
    }
    if(!memory3d_active) {
        special_failed=0; bytes_address=ADDRESS*2;
        lk_agb_cart_write_short(0x4000c2,(uint16_t)bytes_address);
        lk_agb_cart_write_short(0x4000c3,bytes_address>>16);
        lk_agb_cart_write_short(0x4000c4,0x1000);
        lk_agb_cart_write_short(0x4000c5,0x0800);
        lk_agb_cart_write_short(0x4000c6,BUFFER/512);
        lk_agb_cart_write_short(0x4000c7,0);
        lk_agb_cart_write_short(0x4000c0,0x11);
        lk_agb_cart_write_short(0x4000c1,0);
        lk_delay_us(400);
        cart_romRead_begin(0x800);
        memory3d_offset=0; memory3d_active=1;
    }
    left=TRANSFER;
    while(left && !flashgbx_cancelled()) {
        n=left>64?64:left;
        cart_romRead_continue((uint16_t *)responBuf,n/2);
        if(!send_packet(responBuf,n)) return;
        left-=n; memory3d_offset+=n;
    }
}
static void set_pins(uint32_t mask, bool high)
{
    /* 盖中盖只有片选、读写和地址数据线，电源、CLK、AUDIO 请求仅应答。 */
    if (mask & 4U) CART_WR(high);
    if (mask & 8U) CART_RD(high);
    if (mask & 16U) CART_CS1(high);
    if (mask & (1UL << 29)) CART_CS2(high);

    uint32_t ad_mask = (mask >> 5) & 0xffffU;
    uint32_t a_mask = (mask >> 21) & 0xffU;
    if (ad_mask) {
        cart_setDirection_ad(1);
        GPIOB->BSRR = high ? ad_mask : ad_mask << 16;
    }
    if (a_mask) {
        cart_setDirection_a(1);
        GPIOA->BSRR = high ? a_mask : a_mask << 16;
    }
}

bool flashgbx_handle(void)
{
    uint16_t len, i, offset;
    uint8_t command, result = 1;
    if (!active) {
        if (busy || cmdBuf_p < 2 || cmdBuf[0] != 0x55 || cmdBuf[1] != 0xaa) return false;
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (cmdBuf_p < 2 || cmdBuf[0] != 0x55 || cmdBuf[1] != 0xaa) {
            __set_PRIMASK(mask);
            return false;
        }
        transaction_id = session_id;
        active = true;
        __set_PRIMASK(mask);
        state_defaults();
    }
    if (transport_fault || !cmdBuf_p) return true;
    len = expected();
    if (cmdBuf_p < len || flashgbx_cancelled()) return true;
    if(pending) {
        command=pending; pending=0;
        if(command==0xaf) {
            FgxState saved_state;
            memcpy(&saved_state,cmdBuf,sizeof(state));
            result=1;
            if(saved_state.word[0]>FGX_READ_LIMIT || saved_state.byte[0]>2 ||
               saved_state.selected_voltage!=1 || saved_state.configured>1 ||
               saved_state.byte[14]>3 || saved_state.byte[15]>1 || saved_state.byte[16]>1 ||
               saved_state.wide[1]>32767000UL ||
               saved_state.bank_count>3 ||
               (saved_state.configured && (saved_state.byte[2]>2 || saved_state.byte[3]<1 || saved_state.byte[3]>12)) || (saved_state.byte[4]>3 || saved_state.byte[4]==2)) {
                memset(&saved_state,0,sizeof(saved_state)); saved_state.selected_voltage=1;
                result=2;
            }
            memcpy(&state,&saved_state,sizeof(state));
            state.auto_ticks=0;
            ack(result);
        } else {
            if(command==0xd3) {
                if(state.byte[3]>=3 && (state.configured || fixed_method(state.byte[3]) || state.byte[3]==7)) result=program_special()?1:2;
                else result=program_flash()?1:2;
            }
            else if(command==0xc6) result=write_eeprom(pending_type)?1:2;
            else if(command==0xc7) result=program_save(pending_type)?1:2;
            else if(command==0xb6 && MODE==1 && TRANSFER<=32 && !(TRANSFER&1) && ADDRESS+TRANSFER/2<=256) {
                special_failed=0; lk_dmg_mbc7_write_eeprom(); result=special_failed?2:1;
            }
            else if(command==0xb7 && MODE==1 && valid_range(TRANSFER,0)) {
                special_failed=0; lk_dmg_mbc6_mmsa_write_flash_bytes(); result=special_failed?2:1;
            }
            else if(command==0xb9) result=dmg_page_write()?1:2;
            else if(command==0xb3 && MODE==1 && valid_range(TRANSFER,0)) { cart_gbcWrite_jr(ADDRESS,cmdBuf,TRANSFER); ADDRESS+=TRANSFER; }
            else if(command==0xc4 && MODE==2 && valid_range(TRANSFER,0)) { cart_ramWrite(ADDRESS,cmdBuf,TRANSFER); ADDRESS+=TRANSFER; }
            else result=2;
            ack(result);
        }
        consume(len); return 1;
    }
    command=cmdBuf[0];
    if(memory3d_active && command!=0xc8) {
        release_bus();
        ADDRESS+=BUFFER/2; memory3d_active=0;
        if(command==0) { consume(len); return 1; }
    }
    /* 流式读取允许 8 KiB；所有跟随原始输入载荷的写命令仍限制为接收缓冲可容纳的 4 KiB。 */
    if(TRANSFER>FGX_LIMIT && (command==0xb3 || command==0xc4 || command==0xd3 || command==0xc6 ||
       command==0xc7 || command==0xb6 || command==0xb7 || command==0xb9)) {
        ack(2);transport_fault=1;consume(len);return 1;
    }
    switch(command) {
    case 0xa0: ack(1); break;
    case 0x55:
        if(cmdBuf[1]==0xaa) {
            for(i=0;i<sizeof(probe_reply)-1;i++) reply[i]=probe_reply[i];
            send_packet(reply,sizeof(probe_reply)-1);
        } else ack(2);
        break;
    case 0x4c: ack(cmdBuf[1]==0x4b?0xff:2); break;
    case 0xa1: query_info(); break;
    case 0xa2:
        if(MODE!=2) state.configured=0;
        MODE=2;
        /* 固定供电板由 JR 在读取卡头时发送 C9。 */
        ack(1); break;
    case 0xa3:
        if(MODE!=1) state.configured=0;
        MODE=1; ack(1); break;
    case 0xa4: case 0xa5:
        /* A5 的 ACK 只完成 FlashGBX 请求，物理供电固定为 3.3V。 */
        ack(1); break;
    case 0xa6: ack(set_variable(cmdBuf[1],be32(2),be32(6))?1:2); break;
    case 0xad: put32(get_variable(cmdBuf[1],be32(2))); send_packet(reply,4); break;
    case 0xa8: release_bus(); ack(1); break;
    case 0xa9: ack(1); break;
    case 0xab: state.byte[14]=1; ack(1); break;
    case 0xac: state.byte[14]=0; ack(1); break;
    case 0xb1: case 0xc1: case 0xc3: read_stream(command); break;
    case 0xb5:
        if(MODE!=1 || !TRANSFER || TRANSFER>32 || (TRANSFER&1) || ADDRESS+TRANSFER/2>256) { ack(2); break; }
        special_failed=0; lk_dmg_mbc7_read_eeprom(); break;
    case 0xc8: read_3d(); break;
    case 0xc5: read_eeprom(cmdBuf[1]); break;
    case 0xc6: case 0xc7:
        if(!TRANSFER || TRANSFER>FGX_LIMIT) { ack(2); transport_fault=1; }
        else { pending=command; pending_type=cmdBuf[1]; }
        break;
    case 0xca:
        if(MODE!=2) { ack(2); break; }
        gba_rtc_read(reply); send_packet(reply,8); break;
    case 0xb2: case 0xd1:
        if(MODE!=1) ack(2); else { write_unit(be32(1),cmdBuf[5],command==0xd1); ack(1); } break;
    case 0xc2: case 0xd2:
        if(MODE!=2) ack(2); else { write_unit(be32(1),be16(5),1); ack(1); } break;
    case 0xb3: case 0xc4: case 0xd3: case 0xb6: case 0xb7: case 0xb9:
        /* 长度无效时无法定位后续数据边界，必须隔离到重连，避免把数据当写命令。 */
        if(!TRANSFER || TRANSFER>FGX_LIMIT) { ack(2); transport_fault=1; }
        else pending=command;
        break;
    case 0xd4:
        if(MODE!=1 && MODE!=2) { ack(2); break; }
        for(i=0;i<cmdBuf[2];i++) { offset=3+i*6; write_unit(be32(offset),be16(offset+4),cmdBuf[1]!=0); }
        ack(1); break;
    case 0xa7:
        state.configured=0;
        if(cmdBuf[1]>2 || cmdBuf[2]<1 || cmdBuf[2]>12 || cmdBuf[3]>3 || cmdBuf[3]==2) { ack(2); break; }
        if(cmdBuf[1]==1 && cmdBuf[2]<=2 && !amd_commands_valid()) { ack(2); break; }
        if(cmdBuf[1]==2 && cmdBuf[2]<=2 && ((cmdBuf[2]==2 && ((be16(8)!=0xe8 && be16(8)!=0xe9 && be16(8)!=0xea) || be16(26)!=0xd0)) ||
           (cmdBuf[2]==1 && be16(8)!=0x40 && be16(8)!=0x10 && !((be16(8)==0x70 && (be16(14)==0x10 || be16(14)==0x40)) || (be16(8)==0xb0 && be16(14)==0x10))))) { ack(2); break; }
        state.byte[2]=cmdBuf[1]; state.byte[3]=cmdBuf[2]; state.byte[4]=cmdBuf[3];
        for(i=0;i<6;i++) { offset=4+i*6; state.commands[i].address=be32(offset); state.commands[i].value=be16(offset+4); }
        state.configured=1; ack(1); break;
    case 0xb8:
        if(cmdBuf[1]>3) { ack(2); break; }
        for(i=0;i<cmdBuf[1];i++) if(cmdBuf[6+i*5]>1 ||
           be32(2+i*5)>(cmdBuf[6+i*5]?255UL:65535UL)) result=2;
        if(result==2) { ack(2); break; }
        state.bank_count=cmdBuf[1];
        for(i=0;i<state.bank_count;i++) { state.bank_address[i]=be32(2+i*5); state.bank_mode[i]=cmdBuf[6+i*5]; }
        ack(1); break;
    case 0xb4:
        /* FlashGBX 的卡带分析在 AGB 模式也会发送 B4 清理。
           /RD、/WR 保持高电平，GBA 的 /CS2 脉冲不会形成存档写周期。 */
        CART_RD(1); CART_WR(1);
        CART_CS2(0);
        lk_delay_us(5000);
        CART_CS2(1); ack(1); break;
    case 0xc9:
        if(MODE!=2) { ack(2); break; }
        special_failed=0; lk_agb_bootup_sequence(); ack(special_failed?2:1); break;
    case 0xd5: crc_rom(be32(1)); break;
    case 0xae: send_packet((uint8_t *)&state, sizeof(state)); break;
    case 0xaf: pending=command; break;
    case 0xf2: ack(1); break;
    case 0xf3: release_bus(); ack(1); break;
    case 0xf4: ack(FGX_CARD_POWER); break;
    case 0xfe: ack((uint8_t)~cmdBuf[1]); break;
    case 0xf5: set_pins(be32(1),cmdBuf[5]!=0); ack(1); break;
    case 0xf6: ack(0); break; /* 没有安装实体模式/插卡检测开关，握手也不声明这些能力。 */
    case 0x30: ack(1); break;
    case 0xf1:
        /* 未支持的流式命令可能紧跟原始数据，隔离到重连，避免当作新命令执行。 */
        ack(2); transport_fault=1; break;
    default: ack(2); break;
    }
    consume(len); return 1;
}