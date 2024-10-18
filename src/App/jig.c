#include "jig.h"
#include "stdlib.h"

uint8_t connId = 0;

calSend_t calSend = {   .state = JIG_CAL_SEND_IDLE,
                        .sentOffset = 0};

char scanBuff[100] = {0};

char gtfStateChr[6][10] = { "NONE",
                            "COLLECT",
                            "ORIENT",
                            "REAL_TIME",
                            "RUN_CAL",
                            "SLEEP"};
char colorStateChr[2][10] = {"BLUE","GREEN"};
char calStateChr[3][10] = {"UNCAL","CAL_RUN","CAL_OK"};

uint8_t retryConn = 0;
bool tryConn = false;

void calJigMain(void){

    Calibracao();
    parseProtocol();
    processScan();
    sendCal();

    if(tryConn){
        
        tryConn = false;
        bleConnect(&bleList[connId].addr);
    }

}

void parseProtocol(void){

    uint8_t rdBuf[64] = {0};
    char buf64[128] = {0};
    uint32_t oLen = 0;
    uint32_t sizeRead = 0;
    uint16_t crcCalc = 0xffff;
    uint16_t offset = 0;
    int32_t rc = 0;
    /*uint16_t commaPos = 0;
    
    uint16_t fieldSize = 0;
    uint64_t tempVar = 0;*/

    if(parseCmd){

        printk("Recv.: %s\n", bufferCmdRx);

        if(bufferCmdRx[0] == 'S'){

            scanState = SCAN_INIT;

        }else if(bufferCmdRx[0] == 'C'){

            bt_le_scan_stop();
            scanState = SCAN_DONE;

            connId = bufferCmdRx[1] - '0';
            if(bleListPos && (connId < bleListPos)){
                bleConnect(&bleList[connId].addr);
                retryConn = RETRY_CONN_MAX;
            }

        }else if(bufferCmdRx[0] == 'D'){
            bleDisconnect(default_conn);
        }else if(bufferCmdRx[0] == 'R'){
            bt_gtf_read_status(&gtf_client);
        }else if(bufferCmdRx[0] == 'W'){

            calSend.state = JIG_CAL_SEND_START;
            calSend.sentOffset = 0;

        }else if(bufferCmdRx[0] == 'M'){

            if(bufferCmdRx[1] == 'R'){

            }else if(bufferCmdRx[1] == 'B'){

                for(uint32_t i = 0; i<CAL_FULL_SIZE; i+=sizeRead){

                    sizeRead = CAL_FULL_SIZE - i > 12?
                            12 : CAL_FULL_SIZE - i;
                    mb85rs1ReadData(i + INC_MATRIX_ADDR, rdBuf, sizeRead); 
                    base64_encode(buf64, sizeof(buf64), &oLen, rdBuf, sizeRead);
                    printk("%s", buf64);
                }

            }

        }else if(bufferCmdRx[0] == 'P'){

            if(bufferCmdRx[1] == 'S'){
            
                /*rdBuf[0] = GTF3_COLLECT;
                sscanf(bufferCmdRx + 2, "%4x", &rdBuf[1]);
                sscanf(bufferCmdRx + 6, "%1x", &rdBuf[3]);*/
                oLen = cmdIndex - 3;
                memcpy(rdBuf, bufferCmdRx + 2, oLen);
                crcCalc = crc16_itu_t(crcCalc, rdBuf, oLen);
                memcpy(rdBuf + oLen, &crcCalc, 2);
                oLen+=2;
                
                base64_encode(buf64, sizeof(buf64), &oLen, rdBuf, oLen);
                strcat(buf64, "\n");

                oLen++;

                atomic_clear_bit(&gtf_client.state, GTF_C_CAL_WRITE_PENDING);

                while(offset < oLen){
                    
                    /*if(atomic_test_bit(&gtf_client.state, GTF_C_CAL_WRITE_PENDING)){
                        __NOP();
                    }else*/{
                        if((oLen - offset) > currMTU - 3){
                            //rc = bt_gatt_notify(NULL, &vnd_svc.attrs[11], accValues + offset, currMTU - 3);
                            rc = bt_gtf_write_status(&gtf_client, buf64 + offset, currMTU - 3);
                            if(!rc){
                                offset += currMTU - 3;
                            }else if(rc == -EALREADY){

                            }else{
                                offset = oLen;
                            }
                        }else{
                            //rc = bt_gatt_notify(NULL, &vnd_svc.attrs[11], accValues + offset, oLen - offset);
                            rc = bt_gtf_write_status(&gtf_client, buf64 + offset, oLen - offset);
                            if(!rc){
                                offset += oLen - offset;
                            }else if(rc == -EALREADY){

                            }else{
                                offset = oLen;
                            }
                        }
                    }
                }
                //bt_gtf_write_status(&gtf_client, buf64, oLen);

            } else if(bufferCmdRx[1] == 'O'){

            }

        }

        parseCmd = false;
        cmdIndex = 0;
        memset(bufferCmdRx, 0, sizeof(bufferCmdRx));

        uart_irq_rx_enable(cdc1_dev);

    }
}

void processScan(void){

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
    }
}

void sendCal(void){

    uint8_t rdBuf[64] = {0};
    char buf64[128] = {0};
    uint32_t oLen = 0;
    uint32_t sizeRead = 0;
    int32_t res = 0;

    switch (calSend.state){

        case JIG_CAL_SEND_IDLE:{

        }break;

        case JIG_CAL_SEND_START:{

            printk("%s", "START_CAL\n");
            strcat((char*)rdBuf, "START_CAL");
            base64_encode(buf64, sizeof(buf64), &oLen, rdBuf, 9);
            strcat(buf64, "\n");
            oLen++;

            res = bt_gtf_write_cal(&gtf_client, buf64, oLen);

            if(!res){

                calSend.state = JIG_CAL_SEND_DATA;

            }else if(res == -ENOTCONN){

                calSend.state = JIG_CAL_SEND_IDLE;

            }

        }break;

        case JIG_CAL_SEND_DATA:{

            /*sizeRead = CAL_FULL_SIZE - calSend.sentOffset > sizeof(rdBuf)?
                        sizeof(rdBuf) : CAL_FULL_SIZE - calSend.sentOffset;*/
            sizeRead = CAL_FULL_SIZE - calSend.sentOffset > 12?
                        12 : CAL_FULL_SIZE - calSend.sentOffset;
            mb85rs1ReadData(calSend.sentOffset + INC_MATRIX_ADDR, rdBuf, sizeRead); 
            base64_encode(buf64, sizeof(buf64), &oLen, rdBuf, sizeRead);
            strcat(buf64, "\n");
            oLen++;

            res = bt_gtf_write_cal(&gtf_client, buf64, oLen);
            
            if(!res){

                calSend.sentOffset += sizeRead;
                if(calSend.sentOffset == CAL_FULL_SIZE){
                    calSend.state = JIG_CAL_SEND_CRC;
                }
                printk("Pkg size: %u; Sent: %u; Total: %u\n", sizeRead, calSend.sentOffset, CAL_FULL_SIZE);
            }else if(res == -ENOTCONN){

                calSend.state = JIG_CAL_SEND_IDLE;

            }

        }break;

        case JIG_CAL_SEND_CRC:{

            sprintf((char*)rdBuf,"END_CAL");
            mb85rs1ReadData(CRC_BYTES_ADDR, rdBuf + 7, 2);
            base64_encode(buf64, sizeof(buf64), &oLen, rdBuf, 9);
            strcat(buf64, "\n");
            oLen++;

            res = bt_gtf_write_cal(&gtf_client, buf64, oLen);

            if(res != -EALREADY){

                calSend.state = JIG_CAL_SEND_IDLE;

            }

        }break;

        case JIG_CAL_READ_STATUS:{

        }break;

        case JIG_CAL_CHECK_CRC:{

        }break;

        default:{

        }break;
    }

}


/******************************************************************************
 Function: getStringPosition
 Description: Return relative position of a string in a bigger string (needle in a haystack)
 Arguments: haystack, needle
 Return: position
 *******************************************************************************/
uint16_t getStringPosition(char *haystack, uint8_t *needle) {

	uint16_t position = 0;

	char *occurency = strstr(haystack, (const char*) needle);

	if (occurency) {
		position = occurency - haystack;
		return position;
	} else {
		return 0;
	}

}
/******************************************************************************/

/********************************************************************************
 Function: ValueToAscii
 Description: Convert integer raw value to ASCII decimal or hexadecimal formatted string
 Arguments: pointerASCII, value, pointerSize, format
 Return: none
 ********************************************************************************/
void valueToAscii(uint8_t *pointerASCII, int64_t value, uint16_t pointerSize,
		uint8_t format) {

	uint64_t temp = 0;
	uint64_t mask = 0xf;
	bool sign = value <0? 1 : 0;

	value = abs(value);

	if (format) {                    //If desired return is in ASCII Hexadecimal
		for (uint64_t i = 0; i < pointerSize; i++) {

			pointerASCII[i] = (value & (mask << ((pointerSize - i - 1) * 4)))
											>> ((pointerSize - i - 1) * 4);
			temp = (value & (0xf << ((pointerSize - i - 1) * 4)))
											>> ((pointerSize - i - 1) * 4);

		}

	} else {                             //If desired return is in ASCII Decimal

		if(sign){
			pointerASCII[0] = '-';
		}

		for (uint64_t i = 0 + sign; i < pointerSize; i++) {
			temp = (uint64_t) (power10(10, pointerSize - i - 1));
			pointerASCII[i] = value / temp;
			value -= (pointerASCII[i] * temp);
		}
	}

	for (uint64_t i = 0 + sign; i < pointerSize; i++) {

		if (/*pointerASCII[i] >=0 && */pointerASCII[i] <= 9) {

			pointerASCII[i] = pointerASCII[i] + 0x30;

		} else if (pointerASCII[i] >= 10 && pointerASCII[i] <= 15) {

			pointerASCII[i] = pointerASCII[i] + 0x37;

		} else {

			pointerASCII[i] = '0';

		}
	}

}
/*******************************************************************************/


/********************************************************************************
 Function: asciiToVar
 Description: Convert ASCII formated HEX string to raw HEX value
 Arguments: pointerASCII, pointerSize
 Return: var
 ********************************************************************************/
uint64_t asciiToVar(uint8_t *pointerASCII, uint16_t pointerSize, bool format) {

	uint8_t tempVar = 0;
	uint64_t var = 0;
	uint16_t powCount = pointerSize - 1;

	if (format) {

		for (uint16_t i = 0; i < pointerSize; i += 1) {

			if (pointerASCII[i] >= '0' && pointerASCII[i] <= '9') {

				tempVar = (pointerASCII[i] - 0x30);

			} else if (pointerASCII[i] >= 'A' && pointerASCII[i] <= 'F') {

				tempVar = (pointerASCII[i] - 0x37);

			} else if (pointerASCII[i] >= 'a' && pointerASCII[i] <= 'f') {

				tempVar = (pointerASCII[i] - 0x57);

			} else {

				tempVar = 0;

			}
			var += tempVar << (4 * powCount--);
		}
	} else {

		for (uint16_t i = 0; i < pointerSize; i += 1) {

			if (pointerASCII[i] >= '0' && pointerASCII[i] <= '9') {

				tempVar = (pointerASCII[i] - 0x30);

			} else {

				tempVar = 0;

			}
			var += tempVar * power10(10, powCount--);
		}
	}
	return var;
}
/*******************************************************************************/

/********************************************************************************
 Function: power10
 Description: Calculate power of "expoent" of given "base"
 Arguments: base, expoent
 Return: var
 ********************************************************************************/
uint64_t power10(uint8_t base, uint8_t expoent) {

	uint64_t ret = 1;

	for(uint8_t i = 0; i<expoent; i++){
		ret *= base;
	}
	return ret;
}
/*******************************************************************************/

/*******************************************************************************
Function: getDecSize
Description: Returns the number of digits of a given integer value
Arguments: val
Return: size
*******************************************************************************/
uint8_t getDecSize(uint64_t val){

  uint8_t size = 1;

  while(val > 9){

    val /=10;
    size++;
  }

  return size;
}
/******************************************************************************/