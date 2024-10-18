#ifndef APP_BLEFUNC_H
#define APP_BLEFUNC_H

#include "../main.h"

#define BLE_LIST_SIZE   5

typedef enum{

    SCAN_STOPPED,
    SCAN_INIT,
    SCAN_SCANNIG,
    SCAN_DONE

}scanState_t;

extern scanState_t scanState;

typedef struct{

    char name[30];
    char rssi[6];
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_t addr;

}bleList_t;

typedef enum{

    BLE_DISCONNECTED,
    BLE_CONNECTING,
    BLE_DISCOVERY,
    BLE_CONNECTED,

}bleConnState_t;

typedef struct {

    bleConnState_t state;
    char addr_str[BT_ADDR_LE_STR_LEN];

}bleConn_t;

typedef struct{

    double xRaw;
    double yRaw;
    double zRaw;

    double xCal;
    double yCal;
    double zCal;

}bleSensData_t;

extern bleList_t bleList[BLE_LIST_SIZE];
extern uint8_t bleListPos;
extern struct bt_conn *default_conn;
extern struct bt_gtf_client gtf_client;
extern bleSensData_t bleSensData;
extern bleConn_t bleConn;
extern bool newAccData;
extern uint16_t currMTU;

int bleStart(void);
int gtf_client_init(void);
uint8_t gtfNotifRcv(struct bt_gtf_client *gtf, const uint8_t *data, uint16_t len, uint16_t handle);
void start_scan(void);
int bleConnect(bt_addr_le_t *addr);
int bleDisconnect(struct bt_conn *conn);

#endif /*APP_BLEFUNC_H*/