#ifndef MAIN_H_
#define MAIN_H_

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/sys/byteorder.h>

#include <bluetooth/services/nus.h>
#include <bluetooth/gatt_dm.h>

#include <zephyr/settings/settings.h>

#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/device.h>
#include <hal/nrf_power.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>

#include "time.h"

#include <zephyr/sys/base64.h>
#include <zephyr/sys/crc.h>

#include "App/Memoria.h"
#include "App/Calibration.h"
#include "App/MB85RS1.h"
#include "App/bleFunc.h"
#include "App/gtfSvc_c.h"
#include "App/jig.h"

#define MSG_SIZE 32

extern struct k_msgq uart_msgq;
extern struct device *const uart_dev;
extern struct device *const cdc0_dev;
extern struct device *const cdc1_dev;
extern char rx_buf[MSG_SIZE];

/*#include "App/bleFunc.h"
#include "App/control.h"
#include "App/MB85RS1.h"
#include "App/appThreads.h"*/

#endif