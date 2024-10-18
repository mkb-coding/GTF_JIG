#ifndef APP_MB85RS1_H
#define APP_MB85RS1_H

#include "../main.h"

#define SRAM_CS_ENABLE          gpio_pin_set(device_get_binding("GPIO_1"), 13, 0);
#define SRAM_CS_DISABLE         gpio_pin_set(device_get_binding("GPIO_1"), 13, 1);

#define MB85RS1_CMD_WREN        0x06
#define MB85RS1_CMD_WRDI        0x04
#define MB85RS1_CMD_RDSR        0x05
#define MB85RS1_CMD_WRSR        0x01
#define MB85RS1_CMD_READ        0x03
#define MB85RS1_CMD_WRITE       0x02
#define MB85RS1_CMD_RDID        0x9f
#define MB85RS1_CMD_FSTRD       0x0b
#define MB85RS1_CMD_SLEEP       0xb9

#define MB85RS1_DEVID           0x047f2703

#define MB85RS1_WPEN_EN         (1 << 7)
#define MB85RS1_WPEN_DIS        (0 << 7)

#define MB85RS1_BPNONE          (0 << 2)
#define MB85RS1_BPUPQUART       (1 << 2)
#define MB85RS1_BPUPHALF        (2 << 2)
#define MB85RS1_BPALL           (3 << 2)

#define MB85RS1_WEL_EN          (1 << 1)
#define MB85RS1_WEL_DIS         (0 << 1)

uint8_t mb85rs1TransceiveCmd(uint32_t wLen, uint8_t* wBuf, uint32_t wLen1, uint8_t* wBuf1, uint32_t rLen, uint8_t* rBuf);
uint8_t mb85rs1Init(void);
uint8_t mb85rs1ReadId(uint32_t* pBuf);
uint8_t mb85rs1ReadSr(uint8_t* pBuf);
uint8_t mb85rs1WriteSr(uint8_t* pBuf);
uint8_t mb85rs1WriteEnable(void);
uint8_t mb85rs1WriteDisable(void);
uint8_t mb85rs1ReadData(uint32_t addr, uint8_t* pBuf, uint32_t len);
uint8_t mb85rs1WriteData(uint32_t addr, uint8_t* pBuf, uint32_t len);
uint8_t mb85rs1ReadDataFast(uint32_t addr, uint8_t* pBuf, uint32_t len);
uint8_t mb85rs1SetSleep(bool mode);

#endif