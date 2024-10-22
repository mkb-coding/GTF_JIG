#include "bleFunc.h"

struct bt_conn *default_conn;
bleList_t bleList[BLE_LIST_SIZE] = {0};
uint8_t bleListPos = 0;
scanState_t scanState = SCAN_STOPPED;

struct bt_gtf_client gtf_client;

bleSensData_t bleSensData = {0};
bleConn_t bleConn = {.state = BLE_DISCONNECTED};

bool newAccData = false;
char bufAcc[100] = {0};
uint32_t bufAccPos = 0;
char bufSts[100] = {0};
uint32_t bufStsPos = 0;

uint16_t currMTU = 0;

#define NAME_LEN 30

static bool data_cb(struct bt_data *data, void *user_data){

	char *name = user_data;
	bool ret = false;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		memcpy(name, data->data, MIN(data->data_len, NAME_LEN - 1));
		ret =  false ;
	default:
		ret =  true;
	}

	return ret;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad){

	char addr_str[BT_ADDR_LE_STR_LEN];
	char name[30] = {0};
	bool newEntry = true;

	if (default_conn) {
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	//printk("Device found: %s (RSSI %d)\n", addr_str, rssi);

	//seting up the bt_data_parse function and prints it. 
	//
	bt_data_parse(ad, data_cb, name);
	//printk("Device found: %s \n", name);

	if((memcmp(name, "GTF_3", 3)) || (name[0] == 0)){
		
		newEntry = false;
		return;
	}

	for(uint8_t i = 0; i <= bleListPos; i++){

		/*if( (!memcmp(bleList[i].name, name, sizeof(name))) && 
			(!memcmp(name, "GTF_3", 3)) && (name[0])){
			newEntry = false;
		}*/
		if (!memcmp(bleList[i].name, name, sizeof(name))){
			newEntry = false;
		}

	}

	if(newEntry){
	
		memcpy(bleList[bleListPos].addr_str, addr_str, sizeof(addr_str));
		memcpy(bleList[bleListPos].name, name, strlen(name));
		sprintf((char*)bleList[bleListPos].rssi,"%d", rssi);
		
		bleList[bleListPos].addr.type = addr->type;
		bleList[bleListPos].addr.a = addr->a;
		
		bleListPos++;

		printk("Device found: ID:%d; %s; MAC: %s \n", bleListPos - 1, name, addr_str);
	}

	if(bleListPos >= BLE_LIST_SIZE){
		bt_le_scan_stop();
		scanState = SCAN_DONE;
	}
	/* connect only to devices in close proximity */
	if (rssi < -70) {
		return;
	}

}

void start_scan(void){

	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

int bleConnect(bt_addr_le_t *addr){

	int err;

	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

	/*err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_PARAM(BT_CONN_LE_OPT_NONE, BT_GAP_SCAN_FAST_INTERVAL, BT_GAP_SCAN_FAST_INTERVAL),
				BT_LE_CONN_PARAM(6, 3200, 0, 3200), &default_conn);*/
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	if (err) {
		printk("Create conn to %s failed (%d)\n", addr_str, err);

	}

	if(!err){
		bleConn.state = BLE_CONNECTING;
		memcpy(bleConn.addr_str, addr, BT_ADDR_LE_STR_LEN);
	}

	return err;
}

int bleDisconnect(struct bt_conn *conn){
	return bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void discover_all_completed(struct bt_gatt_dm *dm, void *ctx){

	struct bt_gtf_client *gtf = ctx;
	int32_t ret = 0;

	printk("Service discovery completed");

	bt_gatt_dm_data_print(dm);

	ret = bt_gtf_handles_assign(dm, gtf);
	if(ret){
		return;
	}

	ret = bt_gtf_subscribe_receive(gtf);
	if(ret){
		return;
	}
	
	bleConn.state = BLE_CONNECTED;
	//memcpy(bleConn.addr_str, addr, BT_ADDR_LE_STR_LEN);

	bt_gatt_dm_data_release(dm);

}

static void discover_all_service_not_found(struct bt_conn *conn, void *ctx){
	printk("No more services\n");
}

static void discover_all_error_found(struct bt_conn *conn, int err, void *ctx){
	printk("The discovery procedure failed, err %d\n", err);
}

static struct bt_gatt_dm_cb discover_all_cb = {
	.completed = discover_all_completed,
	.service_not_found = discover_all_service_not_found,
	.error_found = discover_all_error_found,
};

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	printk("%s: MTU exchange %s (%u)\n", __func__,
	       err == 0U ? "successful" : "failed",
	       bt_gatt_get_mtu(conn));
	currMTU = bt_gatt_get_mtu(conn);
}

static struct bt_gatt_exchange_params mtu_exchange_params = {
	.func = mtu_exchange_cb
};

static int mtu_exchange(struct bt_conn *conn)
{
	int err;

	printk("%s: Current MTU = %u\n", __func__, bt_gatt_get_mtu(conn));

	printk("%s: Exchange MTU...\n", __func__);
	err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
	if (err) {
		printk("%s: MTU exchange failed (err %d)", __func__, err);
	}

	return err;
}

static void connected(struct bt_conn *conn, uint8_t err){

	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {

		printk("Failed to connect to %s (%u). Retries left: %d\n", addr, err, retryConn);
		
		if(retryConn){

			retryConn--;
			//bleConnect(&bleList[connId].addr);
			tryConn = true;

		}
		bt_conn_unref(default_conn);
		default_conn = NULL;

		return;
	}

	if (conn != default_conn) {
		return;
	}

	printk("Connected: %s\n", addr);
	(void)mtu_exchange(conn);

	//bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	//err = bt_gatt_dm_start(conn, NULL, &discover_all_cb, NULL);
	err = bt_gatt_dm_start(conn, BT_UUID_GTF_SERVICE, &discover_all_cb, &gtf_client);
	if (err) {
		printk("Failed to start discovery (err %d)\n", err);
	}

	bleConn.state = BLE_DISCOVERY;
	memcpy(bleConn.addr_str, addr, BT_ADDR_LE_STR_LEN);

}

static void disconnected(struct bt_conn *conn, uint8_t reason){

	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

	bleConn.state = BLE_DISCONNECTED;
	memset(bleConn.addr_str, 0, BT_ADDR_LE_STR_LEN);
	//start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

int bleStart(void){

	int err;

	err = gtf_client_init();
	if (err) {
		printk("GTF Client Init failed (err %d)\n", err);
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	bleConn.state = BLE_DISCONNECTED;
	memset(bleConn.addr_str, 0, BT_ADDR_LE_STR_LEN);
	
	return 0;
}

int gtf_client_init(void){

	int err;
	struct bt_gtf_client_init_param init = {
		.cb = {
			.received = gtfNotifRcv,
			//.sent = ble_data_sent,
		}
	};

	err = bt_gtf_client_init(&gtf_client, &init);
	if (err) {
		printk("GTF Client initialization failed (err %d)\n", err);
		return err;
	}

	printk("GTF Client module initialized\n");
	return err;
}

uint8_t gtfNotifRcv(struct bt_gtf_client *gtf, const uint8_t *data, uint16_t len, uint16_t handle){

	uint8_t ret = BT_GATT_ITER_CONTINUE;
	uint8_t bufRaw[100] = {0};
	uint8_t tmpVal[10] = {0};
	uint32_t oLen = 0;
	uint16_t crcRcv = 0;
	uint16_t crcCalc = 0xffff;
	uint32_t fieldSize = 0;
	uint32_t offsetPos = 0;

	if(handle == gtf->handles.accVal){

		if((bufAccPos + len) < sizeof(bufAcc)){

			memcpy(bufAcc + bufAccPos, data, len);
			bufAccPos += len;

			if(bufAcc[bufAccPos - 1] == '\n'){

				base64_decode(bufRaw, sizeof(bufRaw), &oLen, bufAcc, bufAccPos - 1);
				
				if(oLen >= 4){

					crcCalc = crc16_itu_t(crcCalc, (uint8_t*)bufRaw, oLen - 2);
					crcRcv = (bufRaw[oLen - 1] << 8) + bufRaw[oLen - 2];

					if((crcCalc == crcRcv) || crcRcv == 0xffff){

						fieldSize = getStringPosition((char *)bufRaw + offsetPos, (uint8_t*)";");
						if(fieldSize == sizeof(memoria.OrientadorStatus)){
							memcpy(&memoria.OrientadorStatus, bufRaw + offsetPos, fieldSize);
						}
						offsetPos = offsetPos + fieldSize + 1;

						fieldSize = getStringPosition((char *)bufRaw + offsetPos, (uint8_t*)";");
						if(fieldSize < sizeof(tmpVal)){
							memset(tmpVal, 0, sizeof(tmpVal));
							memcpy(tmpVal, bufRaw + offsetPos, fieldSize);
							bleSensData.xRaw = atof(tmpVal);
						}
						offsetPos = offsetPos + fieldSize + 1;

						fieldSize = getStringPosition((char *)bufRaw + offsetPos, (uint8_t*)";");
						if(fieldSize < sizeof(tmpVal)){
							memset(tmpVal, 0, sizeof(tmpVal));
							memcpy(tmpVal, bufRaw + offsetPos, fieldSize);
							bleSensData.yRaw = atof(tmpVal);
						}
						offsetPos = offsetPos + fieldSize + 1;

						fieldSize = getStringPosition((char *)bufRaw + offsetPos, (uint8_t*)";");
						if(fieldSize < sizeof(tmpVal)){
							memset(tmpVal, 0, sizeof(tmpVal));
							memcpy(tmpVal, bufRaw + offsetPos, fieldSize);
							bleSensData.zRaw = atof(tmpVal);
						}
						offsetPos = offsetPos + fieldSize + 1;

						fieldSize = getStringPosition((char *)bufRaw + offsetPos, (uint8_t*)";");
						if(fieldSize < sizeof(tmpVal)){
							memset(tmpVal, 0, sizeof(tmpVal));
							memcpy(tmpVal, bufRaw + offsetPos, fieldSize);
							bleSensData.xCal = atof(tmpVal);
						}
						offsetPos = offsetPos + fieldSize + 1;

						fieldSize = getStringPosition((char *)bufRaw + offsetPos, (uint8_t*)";");
						if(fieldSize < sizeof(tmpVal)){
							memset(tmpVal, 0, sizeof(tmpVal));
							memcpy(tmpVal, bufRaw + offsetPos, fieldSize);
							bleSensData.yCal = atof(tmpVal);
						}
						offsetPos = offsetPos + fieldSize + 1;

						fieldSize = getStringPosition((char *)bufRaw + offsetPos, (uint8_t*)";");
						if(fieldSize < sizeof(tmpVal)){
							memset(tmpVal, 0, sizeof(tmpVal));
							memcpy(tmpVal, bufRaw + offsetPos, fieldSize);
							bleSensData.zCal = atof(tmpVal);
						}
						offsetPos = offsetPos + fieldSize + 1;

						newAccData = true;
					}	
				}

				bufAccPos = 0;
				memset(bufAcc, 0, sizeof(bufAcc));
			}
			
		}else {
			bufAccPos = 0;
			memset(bufAcc, 0, sizeof(bufAcc));
		}

	} else if(handle == gtf->handles.status){

		if((bufStsPos + len) < sizeof(bufSts)){

			memcpy(bufSts + bufStsPos, data, len);
			bufStsPos += len;

			if(bufSts[bufStsPos - 1] == '\n'){

				//printk("%s", bufSts);

				base64_decode(bufRaw, sizeof(bufRaw), &oLen, bufSts, bufStsPos - 1);

				if(oLen >= 4){										//buffer size must be at least 2 crc bytes + headers
					/*    	uint32_t        collectInterval; 	---4B
							gtf3OpMode_t    opMode;				---1B
							gtf3Color_t     color;				---1B
							uint8_t         offsetAngle[6];		---6B  
							uint8_t         offsetInc[6];		---6B  	
							gtf3ToolFace_t  tf;					---1B 		
							gtf3CalState_t  cal;				---1B
							uint16_t        crc;				---2B
					*/

					crcCalc = crc16_itu_t(crcCalc, (uint8_t*)bufRaw, oLen - 2);
					crcRcv = (bufRaw[oLen - 1] << 8) + bufRaw[oLen - 2];

					if((crcCalc == crcRcv) || crcRcv == 0xffff){

						fieldSize = getStringPosition((char *)bufRaw + offsetPos, (uint8_t*)";");
						offsetPos = offsetPos + fieldSize + 1;

						fieldSize = getStringPosition((char *)bufRaw + offsetPos, (uint8_t*)";");
						if(fieldSize == sizeof(memoria.OrientadorStatus)){
							memcpy(&memoria.OrientadorStatus, bufRaw + offsetPos, fieldSize);
						}
						offsetPos = offsetPos + fieldSize + 1;

						fieldSize = getStringPosition((char *)bufRaw + offsetPos, (uint8_t*)";");
						if(fieldSize == sizeof(memoria.CorOrientador)){
							memcpy(&memoria.CorOrientador, bufRaw + offsetPos, fieldSize);
						}
						offsetPos = offsetPos + fieldSize + 1;

						fieldSize = getStringPosition((char *)bufRaw + offsetPos, (uint8_t*)";");
						if(fieldSize == sizeof(memoria.Calibrado)){
							memcpy(&memoria.Calibrado, bufRaw + offsetPos, fieldSize);
						}
						offsetPos = offsetPos + fieldSize + 1;

						fieldSize = getStringPosition((char *)bufRaw + offsetPos, (uint8_t*)";");
						if(fieldSize == sizeof(memoria.FlagZerouTF)){
							memcpy(&memoria.FlagZerouTF, bufRaw + offsetPos, fieldSize);
						}
						offsetPos = offsetPos + fieldSize + 1;
						
						fieldSize = getStringPosition((char *)bufRaw + offsetPos, (uint8_t*)";");
						if(fieldSize){
							uint16_t tempCrc = asciiToVar(bufRaw + offsetPos, fieldSize, 0);
							memoria.CRC[0] = (uint8_t)tempCrc;
							memoria.CRC[1] = tempCrc >> 8;
						}
						offsetPos = offsetPos + fieldSize + 1;

						printk("State:%s\nColor:%s\nCal:%s\nCRC:0x%x\nCRC_SW:0x%x\n", 
								gtfStateChr[memoria.OrientadorStatus - '0'],
								colorStateChr[memoria.CorOrientador - '0'],
								calStateChr[memoria.Calibrado - '0'],
								(memoria.CRC[1] << 8) + memoria.CRC[0],
								(memoria.CRC[0] << 8) + memoria.CRC[1]);
						}

					bufStsPos = 0;
					memset(bufSts, 0, sizeof(bufSts));
				}
				ret = BT_GATT_ITER_STOP;
			}
			
		}else {
			bufStsPos = 0;
			memset(bufSts, 0, sizeof(bufSts));
		}
	}
	
	return ret;
}