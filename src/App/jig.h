#ifndef APP_JIG_H
#define APP_JIG_H

#include "../main.h"

#define RETRY_CONN_MAX  4       //Total atempts will be RETRY_CONN_MAX + 1

typedef enum{

    JIG_CAL_SEND_IDLE,
    JIG_CAL_SEND_START,
    JIG_CAL_SEND_DATA,
    JIG_CAL_SEND_CRC,
    JIG_CAL_READ_STATUS,
    JIG_CAL_CHECK_CRC

}calSendState_t;

typedef struct{

    calSendState_t state;
    uint32_t sentOffset;

}calSend_t;

typedef enum{

    GTF3_NONE,
    GTF3_COLLECT,
    GTF3_ORIENT,
    GTF3_REAL_TIME,
    GTF3_RUN_CAL,
    GTF3_SLEEP

}gtf3OpMode_t;

extern uint8_t connId;
extern calSend_t calSend;
extern char gtfStateChr[6][10];
extern char colorStateChr[2][10];
extern char calStateChr[3][10];
extern uint8_t retryConn;
extern bool tryConn;

void calJigMain(void);
void parseProtocol(void);
void processScan(void);
void sendCal(void);
uint16_t getStringPosition(char *haystack, uint8_t *needle);
void valueToAscii(uint8_t *pointerASCII, int64_t value, uint16_t pointerSize, uint8_t format);
uint64_t asciiToVar(uint8_t *pointerASCII, uint16_t pointerSize, bool format);
uint64_t power10(uint8_t base, uint8_t expoent);
uint8_t getDecSize(uint64_t val);

#endif