/*
 * Copyright (c) 2022 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "main.h"

/* change this to any other UART peripheral if desired */
//#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define UART_DEVICE_NODE DT_CHOSEN(cdc_acm_uart0)

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
struct device *const cdc0_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));
struct device *const cdc1_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart1));
/* receive buffer used in UART ISR callback */
char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

int main(void)
{
	int ret = 0;

	//Check if VCC is set to Default
	if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) ==
    (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos))
	{
		//Enable Writing to NVMC
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

		//Set VCC to 3V3
		NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
							(UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos);

		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

		// System reset is needed to update UICR registers.
		NVIC_SystemReset();
	}

	/*char tx_buf[MSG_SIZE];

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	// configure interrupt and callback to receive data 
	ret = uart_irq_callback_user_data_set(uart_dev, uartCalCb, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(uart_dev);

	print_uart("Hello! I'm your echo bot.\r\n");
	print_uart("Tell me something and press enter:\r\n");
*/
	//struct device *dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(cdc0_dev)) {
		//LOG_ERR("CDC ACM device not ready");
		return 0;
	}

	uart_irq_callback_set(cdc0_dev, uartCmdCb);
	// Enable rx interrupts 
	uart_irq_rx_enable(cdc0_dev);

	//dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(cdc1_dev)) {
		//LOG_ERR("CDC ACM device not ready");
		return 0;
	}

	uart_irq_callback_set(cdc1_dev, uartCalCb);
	// Enable rx interrupts 
	uart_irq_rx_enable(cdc1_dev);

	ret = usb_enable(NULL);

	if (ret != 0) {
		//LOG_ERR("Failed to enable USB");
		return 0;
	}

	bleStart();
	//bleConn.state = BLE_CONNECTED;
	//strcpy(bleConn.addr_str, "DD:03:12:4A:18:84");
	mb85rs1Init();

	//char scanBuff[100] = {0};

	while(1){
		calJigMain();
		/*Calibracao();
		if(parseCmd){

			printk("Recv.: %s", bufferCmdRx);

			if(bufferCmdRx[0] == 'S'){

				scanState = SCAN_INIT;

			}else if(bufferCmdRx[0] == 'C'){

				bt_le_scan_stop();
				scanState = SCAN_DONE;

				uint8_t connId = bufferCmdRx[1] - '0';
				if(bleListPos && (connId < bleListPos)){
					bleConnect(&bleList[connId].addr);
				}

			}else if(bufferCmdRx[0] == 'D'){
				bleDisconnect(default_conn);
			}else if(bufferCmdRx[0] == 'R'){
				bt_gtf_read_status(&gtf_client);
			}else if(bufferCmdRx[0] == 'W'){
				bt_gtf_read_status(&gtf_client);
			}

			parseCmd = false;
			cmdIndex = 0;
			memset(bufferCmdRx, 0, sizeof(bufferCmdRx));

			uart_irq_rx_enable(cdc1_dev);

		}

		if(scanState == SCAN_INIT){

			bleListPos = 0;

			for(uint8_t i = 0; i < BLE_LIST_SIZE; i++){

				memset(bleList[i].name, 0, sizeof((bleList[i].name)));
				memset(bleList[i].rssi, 0, sizeof((bleList[i].rssi)));
				memset(bleList[i].addr_str, 0, sizeof((bleList[i].addr_str)));
				
			}

			start_scan();

			scanState = SCAN_SCANNIG;

		}else if(scanState == SCAN_DONE){
			
			printk(scanBuff, "\r\n----- Found %d devices -----\r\n", bleListPos);
			//printk(scanBuff);

			for(uint8_t i = 0; i < bleListPos; i++){
				printk(scanBuff, "ID: %d - Name: %s - MAC: %s\r\n", i, bleList[i].name, bleList[i].addr_str);
				//printk(scanBuff);
			}

			scanState = SCAN_STOPPED;
		}*/
	}
	// indefinitely wait for input from the user 
	/*while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
		print_uart("Echo: ");
		print_uart(tx_buf);
		print_uart("\r\n");
	}*/
	return 0;
}
