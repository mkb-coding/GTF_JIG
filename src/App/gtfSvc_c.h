#ifndef GTF_SVC_CLIENT_H
#define GTF_SVC_CLIENT_H

#include "../main.h"

/** @brief UUID of the NUS Service. **/
#define BT_UUID_GTF_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define BT_UUID_GTF_SERVICE   BT_UUID_DECLARE_128(BT_UUID_GTF_VAL)

enum {
	GTF_C_INITIALIZED,
	GTF_C_TX_NOTIF_ENABLED,
	GTF_C_CAL_WRITE_PENDING,
	GTF_C_STS_WRITE_PENDING,
	GTF_C_ASYNC_READ_PENDING
};


/** @brief Handles on the connected peer device that are needed to interact with
 * the device.
 */
struct bt_gtf_client_handles {

	/** Handle of the GTF Status characteristic, as provided by a discovery.*/
	uint16_t status;
    /** Handle of the GTF Battery characteristic, as provided by a discovery.*/
	uint16_t battery;
    /** Handle of the CCC descriptor of the GTF Battery characteristic, as provided by a discovery.*/
	uint16_t battery_ccc;
	/** Handle of the GTF ACC Mode characteristic, as provided by a discovery.*/
	uint16_t accMode;
	/** Handle of the GTF ACC Values characteristic, as provided by a discovery.*/
	uint16_t accVal;
    /** Handle of the CCC descriptor of the GTF ACC Values characteristic, as provided by a discovery.*/
	uint16_t accVal_ccc;
	/** Handle of the CCC descriptor of the GTF Calibration characteristic, as provided by a discovery.*/
	uint16_t calData;
    
};

struct bt_gtf_client;

/** @brief GTF Client callback structure. */
struct bt_gtf_client_cb {
	/** @brief Data received callback.
	 *
	 * The data has been received as a notification of the GTF TX
	 * Characteristic.
	 *
	 * @param[in] gtf  GTF Client instance.
	 * @param[in] data Received data.
	 * @param[in] len Length of received data.
	 *
	 * @retval BT_GATT_ITER_CONTINUE To keep notifications enabled.
	 * @retval BT_GATT_ITER_STOP To disable notifications.
	 */
	uint8_t (*received)(struct bt_gtf_client *gtf, const uint8_t *data, uint16_t len, uint16_t handle);

	/** @brief Data sent callback.
	 *
	 * The data has been sent and written to the GTF RX Characteristic.
	 *
	 * @param[in] gtf  GTF Client instance.
	 * @param[in] err ATT error code.
	 * @param[in] data Transmitted data.
	 * @param[in] len Length of transmitted data.
	 */
	void (*sent)(struct bt_gtf_client *gtf, uint8_t err, const uint8_t *data, uint16_t len);

	/** @brief TX notifications disabled callback.
	 *
	 * TX notifications have been disabled.
	 *
	 * @param[in] gtf  GTF Client instance.
	 */
	void (*unsubscribed)(struct bt_gtf_client *gtf);
};

/** @brief GTF Client structure. */
struct bt_gtf_client {

    /** Connection object. */
	struct bt_conn *conn;

    /** Internal state. */
	atomic_t state;

    /** Handles on the connected peer device that are needed to interact with the device. */
	struct bt_gtf_client_handles handles;

	/** GATT read parameters for GTF Status Characteristic. */
	struct bt_gatt_read_params status_read_params;

	/** GATT write parameters for GTF Status Characteristic. */
	struct bt_gatt_write_params status_write_params;

	/** GATT subscribe parameters for GTF Battery Characteristic. */
	struct bt_gatt_subscribe_params battery_notif_params;

    /** GATT subscribe parameters for GTF ACC data Characteristic. */
	struct bt_gatt_subscribe_params acc_notif_params;

    /** GATT write parameters for GTF Calibration Characteristic. */
	struct bt_gatt_write_params cal_write_params;

    /** Application callbacks. */
	struct bt_gtf_client_cb cb;
};

/** @brief GTF Client initialization structure. */
struct bt_gtf_client_init_param {

        /** Callbacks provided by the user. */
	struct bt_gtf_client_cb cb;
};

int bt_gtf_client_init(struct bt_gtf_client *gtf_c, const struct bt_gtf_client_init_param *gtf_c_init);
int bt_gtf_handles_assign(struct bt_gatt_dm *dm, struct bt_gtf_client *gtf_c);
int bt_gtf_subscribe_receive(struct bt_gtf_client *gtf_c);
int bt_gtf_read_status(struct bt_gtf_client *gtf_c);
int bt_gtf_write_status(struct bt_gtf_client *gtf_c, const uint8_t *data, uint16_t len);
int bt_gtf_write_cal(struct bt_gtf_client *gtf_c, const uint8_t *data, uint16_t len);

#endif