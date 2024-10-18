#include "gtfSvc_c.h"

#define BT_UUID_GTF_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

static struct bt_uuid_128 vnd_uuid = BT_UUID_INIT_128(
	BT_UUID_GTF_SERVICE_VAL);

static struct bt_uuid_128 vnd_read_status = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));

static struct bt_uuid_128 vnd_read_battery = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2));

static struct bt_uuid_128 vnd_read_acc_settings = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef5));

static struct bt_uuid_128 vnd_read_acc_values = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef6));

static struct bt_uuid_128 vnd_read_time_settings = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef7));

static struct bt_uuid_128 vnd_read_calibration = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef8));

static uint8_t recvNotifAcc(struct bt_conn *conn,
			struct bt_gatt_subscribe_params *params,
			const void *data, uint16_t length){

	struct bt_gtf_client *gtf;

	/* Retrieve GTF Client module context. */
	gtf = CONTAINER_OF(params, struct bt_gtf_client, acc_notif_params);

	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0;
		atomic_clear_bit(&gtf->state, GTF_C_TX_NOTIF_ENABLED);
		if (gtf->cb.unsubscribed) {
			gtf->cb.unsubscribed(gtf);
		}
		return BT_GATT_ITER_STOP;
	}

	//printk("[NOTIFICATION] data %p length %u\n", data, length);
	//printk("[NOTIFICATION] data %s length %u\n", (uint8_t*)data, length);

	if (gtf->cb.received) {
		return gtf->cb.received(gtf, data, length, params->value_handle);
	}

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t recvNotifBat(struct bt_conn *conn,
			struct bt_gatt_subscribe_params *params,
			const void *data, uint16_t length)
{
	struct bt_gtf_client *gtf;

	/* Retrieve GTF Client module context. */
	gtf = CONTAINER_OF(params, struct bt_gtf_client, battery_notif_params);

	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0;
		atomic_clear_bit(&gtf->state, GTF_C_TX_NOTIF_ENABLED);
		if (gtf->cb.unsubscribed) {
			gtf->cb.unsubscribed(gtf);
		}
		return BT_GATT_ITER_STOP;
	}
	
	//printk("[NOTIFICATION] data %p length %u\n", data, length);
	//printk("[NOTIFICATION] data %s length %u\n", (uint8_t*)data, length);

	if (gtf->cb.received) {
		return gtf->cb.received(gtf, data, length, params->value_handle);
	}

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t recvReadStatus(struct bt_conn *conn, uint8_t err,
				    struct bt_gatt_read_params *params,
				    const void *data, uint16_t length)
{
	struct bt_gtf_client *gtf;
	uint8_t ret = BT_GATT_ITER_STOP;
	/* Retrieve GTF Client module context. */
	gtf = CONTAINER_OF(params, struct bt_gtf_client, status_read_params);

	/*printk("[READ] data %p length %u\n", data, length);
	printk("[READ] data %s length %u\n", (uint8_t*)&data, length);*/

	if (gtf->cb.received) {
		ret = gtf->cb.received(gtf, data, length, params->single.handle);
		if(ret == BT_GATT_ITER_STOP){
			atomic_clear_bit(&gtf->state, GTF_C_ASYNC_READ_PENDING);
		}
	}

	return ret;
}

static void sendCalDone(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params){

	struct bt_gtf_client *gtf;
	const void *data;
	uint16_t length;

	/* Retrieve NUS Client module context. */
	gtf = CONTAINER_OF(params, struct bt_gtf_client, cal_write_params);

	/* Make a copy of volatile data that is required by the callback. */
	data = params->data;
	length = params->length;

	atomic_clear_bit(&gtf->state, GTF_C_CAL_WRITE_PENDING);
	if (gtf->cb.sent) {
		gtf->cb.sent(gtf, err, data, length);
	}
}

static void sendStatusDone(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params){
	
	struct bt_gtf_client *gtf;
	const void *data;
	uint16_t length;
	char sbuf[50] = {0};
	/* Retrieve NUS Client module context. */
	gtf = CONTAINER_OF(params, struct bt_gtf_client, status_write_params);

	/* Make a copy of volatile data that is required by the callback. */
	data = params->data;
	length = params->length;

	atomic_clear_bit(&gtf->state, GTF_C_STS_WRITE_PENDING);
	if (gtf->cb.sent) {
		gtf->cb.sent(gtf, err, data, length);
	}
	memcpy(sbuf, data, length);
	printk("SENT %u Bytes over BLE: %s\n", length, sbuf);
}

int bt_gtf_client_init(struct bt_gtf_client *gtf_c, const struct bt_gtf_client_init_param *gtf_c_init){
	
	if (!gtf_c || !gtf_c_init) {
		return -EINVAL;
	}

	if (atomic_test_and_set_bit(&gtf_c->state, GTF_C_INITIALIZED)) {
		return -EALREADY;
	}

	memcpy(&gtf_c->cb, &gtf_c_init->cb, sizeof(gtf_c->cb));

	return 0;
}

int bt_gtf_handles_assign(struct bt_gatt_dm *dm, struct bt_gtf_client *gtf_c){

	const struct bt_gatt_dm_attr *gatt_service_attr =
			bt_gatt_dm_service_get(dm);
	const struct bt_gatt_service_val *gatt_service =
			bt_gatt_dm_attr_service_val(gatt_service_attr);
	const struct bt_gatt_dm_attr *gatt_chrc;
	const struct bt_gatt_dm_attr *gatt_desc;

	if (bt_uuid_cmp(gatt_service->uuid, BT_UUID_GTF_SERVICE)) {
		printk("Service not found.\n");
		return -ENOTSUP;
	}
	printk("Getting handles from GTF service.\n");
	memset(&gtf_c->handles, 0xFF, sizeof(gtf_c->handles));

	/* GTF Status Characteristic */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, (const struct bt_uuid*)&vnd_read_status);
	if (!gatt_chrc) {
		printk("Missing GTF Status characteristic.\n");
		return -EINVAL;
	}
	/* GTF Status Value*/
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, (const struct bt_uuid*)&vnd_read_status);
	if (!gatt_desc) {
		printk("Missing GTF Status value descriptor in characteristic.\n");
		return -EINVAL;
	}
	printk("Found handle for GTF Status characteristic.\n");
	gtf_c->handles.status = gatt_desc->handle;
	
	/* GTF Battery Characteristic */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, (const struct bt_uuid*)&vnd_read_battery);
	if (!gatt_chrc) {
		printk("Missing GTF Battery characteristic.\n");
		return -EINVAL;
	}
	/* GTF Battery Value*/
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, (const struct bt_uuid*)&vnd_read_battery);
	if (!gatt_desc) {
		printk("Missing GTF Battery value descriptor in characteristic.\n");
		return -EINVAL;
	}
	printk("Found handle for GTF Battery characteristic.\n");
	gtf_c->handles.battery = gatt_desc->handle;
	/* GTF Battery CCC */
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_GATT_CCC);
	if (!gatt_desc) {
		printk("Missing GTF Battery CCC in characteristic.\n");
		return -EINVAL;
	}
	printk("Found handle for CCC of GTF Battery characteristic.\n");
	gtf_c->handles.battery_ccc = gatt_desc->handle;

	/* GTF ACC Mode Characteristic */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, (const struct bt_uuid*)&vnd_read_acc_settings);
	if (!gatt_chrc) {
		printk("Missing GTF ACC Mode characteristic.\n");
		return -EINVAL;
	}
	/* GTF ACC Mode Value*/
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, (const struct bt_uuid*)&vnd_read_acc_settings);
	if (!gatt_desc) {
		printk("Missing GTF ACC Mode value descriptor in characteristic.\n");
		return -EINVAL;
	}
	printk("Found handle for GTF ACC Mode characteristic.\n");
	gtf_c->handles.accMode = gatt_desc->handle;

	/* GTF ACC Values Characteristic */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, (const struct bt_uuid*)&vnd_read_acc_values);
	if (!gatt_chrc) {
		printk("Missing GTF ACC Values characteristic.\n");
		return -EINVAL;
	}
	/* GTF ACC Value*/
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, (const struct bt_uuid*)&vnd_read_acc_values);
	if (!gatt_desc) {
		printk("Missing GTF ACC Values value descriptor in characteristic.\n");
		return -EINVAL;
	}
	printk("Found handle for GTF ACC Values characteristic.\n");
	gtf_c->handles.accVal = gatt_desc->handle;
	/* GTF ACC CCC */
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_GATT_CCC);
	if (!gatt_desc) {
		printk("Missing GTF ACC Values CCC in characteristic.\n");
		return -EINVAL;
	}
	printk("Found handle for CCC of GTF ACC Values characteristic.\n");
	gtf_c->handles.accVal_ccc = gatt_desc->handle;

	/* GTF Calibration Characteristic */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, (const struct bt_uuid*)&vnd_read_calibration);
	if (!gatt_chrc) {
		printk("Missing GTF Calibration characteristic.\n");
		return -EINVAL;
	}
	/* GTF Calibration Value*/
	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, (const struct bt_uuid*)&vnd_read_calibration);
	if (!gatt_desc) {
		printk("Missing GTF Calibration value descriptor in characteristic.\n");
		return -EINVAL;
	}
	printk("Found handle for Calibration Mode characteristic.\n");
	gtf_c->handles.calData = gatt_desc->handle;

	/* Assign connection instance. */
	gtf_c->conn = bt_gatt_dm_conn_get(dm);
	return 0;
}

int bt_gtf_subscribe_receive(struct bt_gtf_client *gtf_c){
	
	int err;

	if (atomic_test_and_set_bit(&gtf_c->state, GTF_C_TX_NOTIF_ENABLED)) {
		return -EALREADY;
	}
	//Subscribe to Battery notifications
	gtf_c->battery_notif_params.notify = recvNotifBat;
	gtf_c->battery_notif_params.value = BT_GATT_CCC_NOTIFY;
	gtf_c->battery_notif_params.value_handle = gtf_c->handles.battery;
	gtf_c->battery_notif_params.ccc_handle = gtf_c->handles.battery_ccc;
	atomic_set_bit(gtf_c->battery_notif_params.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

	err = bt_gatt_subscribe(gtf_c->conn, &gtf_c->battery_notif_params);
	if (err) {
		printk("Subscribe to BATTERY CHARACTERISTIC failed (err %d)\n", err);
		atomic_clear_bit(&gtf_c->state, GTF_C_TX_NOTIF_ENABLED);
	} else {
		printk("[SUBSCRIBED TO BATTERY CHARACTERISTIC]\n");
	}

	//Subscribe to Acc notifications
	gtf_c->acc_notif_params.notify = recvNotifAcc;
	gtf_c->acc_notif_params.value = BT_GATT_CCC_NOTIFY;
	gtf_c->acc_notif_params.value_handle = gtf_c->handles.accVal;
	gtf_c->acc_notif_params.ccc_handle = gtf_c->handles.accVal_ccc;
	atomic_set_bit(gtf_c->acc_notif_params.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

	err = bt_gatt_subscribe(gtf_c->conn, &gtf_c->acc_notif_params);
	if (err) {
		printk("Subscribe failed (err %d)\n", err);
		atomic_clear_bit(&gtf_c->state, GTF_C_TX_NOTIF_ENABLED);
	} else {
		printk("[SUBSCRIBED TO ACC CHARACTERISTIC]\n");
	}

	/*if(!err){
		//Emulate command reception to enable real time transmission
		uart_irq_rx_disable(cdc1_dev);
        
        memset(bufferCmdRx, 0, sizeof(bufferCmdRx));
		sprintf(bufferCmdRx,"PS3;250;\n");
		cmdIndex = (strlen(bufferCmdRx));
		parseCmd = true;
		
	}*/

	return err;
}

int bt_gtf_read_status(struct bt_gtf_client *gtf_c){

	int err;
	//struct bt_gatt_read_params *p_read_params;

	if (!gtf_c) {
		return -EINVAL;
	}
	if (!gtf_c->conn) {
		return -EINVAL;
	}
	if (atomic_test_and_set_bit(&gtf_c->state, GTF_C_ASYNC_READ_PENDING)) {
		return -EBUSY;
	}

	//p_read_params = &gtf_c->status_read_params;
	gtf_c->status_read_params.func = recvReadStatus;
	gtf_c->status_read_params.handle_count = 1;
	gtf_c->status_read_params.single.handle = gtf_c->handles.status;
	gtf_c->status_read_params.single.offset = 0;

	err = bt_gatt_read(gtf_c->conn, &gtf_c->status_read_params);
	if (err) {
		atomic_clear_bit(&gtf_c->state, GTF_C_ASYNC_READ_PENDING);
	}

	return err;
}

int bt_gtf_write_status(struct bt_gtf_client *gtf_c, const uint8_t *data, uint16_t len){

	int err;

	if (!gtf_c->conn) {
		__NOP();
		err = -ENOTCONN;
		return err;
	}

	if (atomic_test_and_set_bit(&gtf_c->state, GTF_C_STS_WRITE_PENDING)) {
		__NOP();
		err = -EALREADY;
		return err;
	}

	gtf_c->status_write_params.func = sendStatusDone;
	gtf_c->status_write_params.handle = gtf_c->handles.status;
	gtf_c->status_write_params.offset = 0;
	gtf_c->status_write_params.data = data;
	gtf_c->status_write_params.length = len;

	err = bt_gatt_write(gtf_c->conn, &gtf_c->status_write_params);
	if (err) {
		atomic_clear_bit(&gtf_c->state, GTF_C_CAL_WRITE_PENDING);
	}

	return err;
}

int bt_gtf_write_cal(struct bt_gtf_client *gtf_c, const uint8_t *data, uint16_t len){

	int err;

	if (!gtf_c->conn) {
		return -ENOTCONN;
	}

	if (atomic_test_and_set_bit(&gtf_c->state, GTF_C_CAL_WRITE_PENDING)) {
		return -EALREADY;
	}

	gtf_c->cal_write_params.func = sendCalDone;
	gtf_c->cal_write_params.handle = gtf_c->handles.calData;
	gtf_c->cal_write_params.offset = 0;
	gtf_c->cal_write_params.data = data;
	gtf_c->cal_write_params.length = len;

	err = bt_gatt_write(gtf_c->conn, &gtf_c->cal_write_params);
	if (err) {
		atomic_clear_bit(&gtf_c->state, GTF_C_CAL_WRITE_PENDING);
	}

	return err;
}