#ifndef __OWUTILS_H__
#define __OWUTILS_H__

#include "main.h"

#define DELAY_RESET           500       // uS
#define DELAY_PRESENCE        15
#define DELAY_WRITE_0         60
#define DELAY_WRITE_0_PAUSE   10
#define DELAY_WRITE_1         10
#define DELAY_WRITE_1_PAUSE   60
#define DELAY_READ_SLOT       10
#define DELAY_BUS_RELAX       10
#define DELAY_READ_PAUSE      50
#define DELAY_T_CONVERT12     760       // mS
#define DELAY_T_CONVERT11     380       // mS
#define DELAY_T_CONVERT10     190       // mS
#define DELAY_T_CONVERT9      95        // mS
#define DELAY_PROTECTION      2

//#define scrpd_len             9
//#define addr_len              8
//#define max_sensors           8

#define SEARCH_ROM_CMD        0xF0
#define READ_ROM_CMD          0x33
#define MATCH_ROM_CMD         0x55
#define SKIP_ROM_CMD          0xCC
//#define alarm_search_cmd      0xEC
#define CONV_TEMP_CMD	      0x44
#define WRITE_MEM_CMD         0x4E
#define READ_MEM_CMD          0xBE
#define COPY_MEM_CMD          0x48
//#define recall_ee_cmd         0xB8

typedef struct {
    int8_t celsius;
    uint8_t frac;
    uint8_t tH;
    uint8_t tL;
    uint8_t config; // ADC: 1F - 9 bit, 3F - 10 bit, 5F - 11 bit, 7F - 12 bit
    uint8_t res_0xff;
    uint8_t res_none;
    uint8_t res_0x10;
    uint8_t crc;
    uint8_t crc_calc;
} SensorData;

uint8_t OWFirst(void);
uint8_t OWNext(void);
void 	OWTargetSetup(uint8_t family_code);
void 	OWWriteEEPROM(SensorData * sd);
uint8_t OWGetParams(uint8_t addr_cmd, SensorData *sens);


#endif /* __OWUTILS_H__ */
