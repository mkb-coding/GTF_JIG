/*
 * Calibracao.c
 *
 *  Created on: 20/02/2018
 *      Author: filipe.bernardes
 */
#include "calibration.h"

//#define SHORT_CAL

///////////////////variaveis///////////////////////////

char startCal = 0; //indica que esta calibrando ou nao(0 estado normal 1 calibrando)

unsigned char calDadoRx = 0;
unsigned char calIndex = 0;
unsigned char curCalIndex = 0;

unsigned char bufferCalRx[MAX_RX_BUFFER] = {0};
unsigned char bufferCalibration[MAX_BUFFER] = {0};

unsigned char cmdIndex = 0;
unsigned char bufferCmdRx[MAX_RX_BUFFER] = {0};
bool parseCmd = false;

CRC16_u crc16_calc;
CRC16_u crc16_recv;

state_cal_t state_cal = CAL_WAIT;

state_cal_receive_t state_cal_receive = CAL_RECEIVE_NONE;

double value;

unsigned char resetBuffer = 0;

unsigned char vectorCal = 0;
unsigned char matrixCal = 0;

unsigned char CalIsOff = 0;

unsigned short bufIndex = 0;

uint8_t bleUUID[13] = {};
extern uint8_t forceCalFlag;
///////////////////prototipos///////////////////////////

state_cal_receive_t CalReceive();

void CalClearBuffer();

unsigned short crc16c(const unsigned char* data_p, unsigned char length);


/**
 *
 */
void CalOff() {

	if (CalIsOff == 0) {

		uartCalDeInit();

		CalIsOff = 1;
	}
}

/**
 *
 */
void CalOn() {
	//CalIsOff = 1;
	if (CalIsOff == 1) {

		uartCalInit();

		CalIsOff = 0;
	}
}

/**
 *
 */
void 
Calibracao() {

	double xDouble;
	double yDouble;
	double zDouble;

	switch (state_cal) {

	case CAL_WAIT:{

		if (CalReceive() == CAL_RECEIVING && calIndex > 2) { //maior do que dois porque nao existe comando com menos do que isso de tamanho

			memset(bufferCalibration, 0, MAX_BUFFER);

			memcpy(bufferCalibration, bufferCalRx, calIndex);

			if (bufferCalibration[0] == CAL_RECV_PACKAGE) {

				if (bufferCalibration[1] == CAL_CMD_START) {

					state_cal = CAL_START;
					//GPIO_DRV_ClearPinOutput(MEM_VCC);			// liga alimenta��o da memoria.

				} else if (bufferCalibration[1] == CAL_CMD_READ_RAW) {

					state_cal = CAL_READ_RAW;

				} else if (bufferCalibration[1] == CAL_CMD_READ_CAL) {

					state_cal = CAL_READ_CAL;

				} else if (bufferCalibration[1] == CAL_CMD_READ_MEMORY_CONFIG) {

					state_cal = CAL_READ_MEMORY_CONFIG;

				} else if (bufferCalibration[1] == CAL_CMD_READ_STATUS) {

					state_cal = CAL_READ_STATUS;

				} else if (bufferCalibration[1] == CAL_CMD_WRITE_MATRIX_XYZ) {
#ifdef SHORT_CAL
					curCalIndex = calIndex;
					state_cal = CAL_WRITE_MATRIX_XYZ;
#else
					if (calIndex == 76) { //size + header
						curCalIndex = calIndex;
						state_cal = CAL_WRITE_MATRIX_XYZ;
					}
#endif

				} else if (bufferCalibration[1] == CAL_CMD_WRITE_VECTOR_XYZ) {
#ifdef SHORT_CAL
					curCalIndex = calIndex;
					state_cal = CAL_WRITE_VECTOR_XYZ;
#else
					if (calIndex == 28) { //size + header
						curCalIndex = calIndex;
						state_cal = CAL_WRITE_VECTOR_XYZ;
					}
#endif
				} else if (bufferCalibration[1] == CAL_CMD_WRITE_CALIBRATION) {
#ifdef SHORT_CAL
					curCalIndex = calIndex;
					state_cal = CAL_WRITE_CALIBRATION;
#else
					if (calIndex == 54) { //size + header
						curCalIndex = calIndex;
						state_cal = CAL_WRITE_CALIBRATION;
					}
#endif
				} else if (bufferCalibration[1] == CAL_CMD_WRITE_FINAL_CRC) {


						state_cal = CAL_WRITE_FINAL_CRC;


				} else if (bufferCalibration[1] == CAL_CMD_RESET) {

					state_cal = CAL_RESET;

				} else if (bufferCalibration[1] == CAL_CMD_FINISH) {

					state_cal = CAL_FINISH;

				} else if (bufferCalibration[1] == CAL_CMD_OT_VER){

					state_cal = CAL_OT_VER;

				} else if (bufferCalibration[1] == CAL_CMD_READ_CRC){

					state_cal = CAL_READ_CRC;
				} else if (bufferCalibration[1] == CAL_CMD_WRITE_FLASH){
					state_cal = CAL_WRITE_FLASH;
				}

				if (state_cal != CAL_WAIT) {

					bufferCalibration[0] = CAL_SENT_PACKAGE; //troca o primeiro byte pelo codigo de resposta

					CalClearBuffer();
				}

			} else {

				CalClearBuffer();
			}

			resetBuffer++;

			if (resetBuffer == 10) {

				resetBuffer = 0;

				CalClearBuffer();
			}

		} else if (CalReceive() == CAL_RECEIVED_OVERFLOW) {

			//Se estourou o tamanho do buffer e n�o encontrou nada ent�o limpa

			CalClearBuffer();
		}

		k_msleep(50);

	}break;

	case CAL_START:{

		matrixCal = 0;
		vectorCal = 0;

		bufIndex = 0;

		//if (bleUUID[0] != 0x00) {
		if(bleConn.state == BLE_CONNECTED){

			//memcpy(&bufferCalibration[2], bleUUID, 12);
			memcpy(&bufferCalibration[2], bleConn.addr_str, 2);
			memcpy(&bufferCalibration[4], bleConn.addr_str + 3, 2);
			memcpy(&bufferCalibration[6], bleConn.addr_str + 6, 2);
			memcpy(&bufferCalibration[8], bleConn.addr_str + 9, 2);
			memcpy(&bufferCalibration[10], bleConn.addr_str + 12, 2);
			memcpy(&bufferCalibration[12], bleConn.addr_str + 15, 2);
			

			/////////////////////CRC16/////////////////////

			crc16_calc.crc16 = crc16c(&bufferCalibration[1], 13);

			memcpy(&bufferCalibration[14], crc16_calc.crc16_bytes, 2);

			BleOff();

			calSendData( bufferCalibration, 16);

			/////***********/////adxl_355_Set_Power_Ctl(ADXL_355_DRDY_ON, ADXL_355_TEMP_OFF, ADXL_355_MEASURE);
		}

		state_cal = CAL_WAIT;

	}break;

	case CAL_READ_RAW:{
		k_busy_wait(WAIT_A_MSEC * 300);
		//faz a leitura do acelerometro sem aplicacao da calibracao
		readXYZ();

		//TODO: falta condicional para tratativa de sucesso/falha na aquisição do pacote com valores do XYZ

		inc_GRAV_FIFO_raw(&xDouble, &yDouble, &zDouble);
		memcpy(&bufferCalibration[2], &xDouble, 8);
		memcpy(&bufferCalibration[10], &yDouble, 8);
		memcpy(&bufferCalibration[18], &zDouble, 8);

		/////////////////////CRC16/////////////////////

		crc16_calc.crc16 = crc16c(&bufferCalibration[1], 25);

		memcpy(&bufferCalibration[26], crc16_calc.crc16_bytes, 2);

		/////////////////////Send Serial///////////////

		calSendData( bufferCalibration, 28);
		//LPUART_DRV_SendData(FSL_LPUART_CAL, bufferCalibration, 28);

		state_cal = CAL_WAIT;

	}break;

	case CAL_READ_CAL:{
		k_busy_wait(WAIT_A_MSEC * 300);
		//faz a leitura do acelerometro com aplicacao da calibracao
		readXYZ();

		value = inc_GRAV_X();

		memcpy(&bufferCalibration[2], &value, 8);

		value = inc_GRAV_Y();

		memcpy(&bufferCalibration[10], &value, 8);

		value = inc_GRAV_Z();

		memcpy(&bufferCalibration[18], &value, 8);

		/////////////////////CRC16/////////////////////

		crc16_calc.crc16 = crc16c(&bufferCalibration[1], 25);

		memcpy(&bufferCalibration[26], crc16_calc.crc16_bytes, 2);

		/////////////////////Send Serial///////////////

		calSendData( bufferCalibration, 28);

		state_cal = CAL_WAIT;

	}break;

	case CAL_READ_MEMORY_CONFIG:

		//faz a leitura da memoria de configuracao
		
		Memoria_ReadData(&memoria);


		memcpy(&bufferCalibration[2], &memoria.CorOrientador, 1);
		memcpy(&bufferCalibration[3], &memoria.AnguloOffset, 6);
		memcpy(&bufferCalibration[9], &memoria.IncOffset, 6);
		memcpy(&bufferCalibration[15], &memoria.FlagZerouTF, 1);
		memcpy(&bufferCalibration[16], &memoria.OrientadorStatus, 1);
		memcpy(&bufferCalibration[17], &memoria.Calibrado, 1);

		memcpy(&bufferCalibration[18], &inclinacao.matrix, MAX_BYTE_MATRIX_XYZ);
		memcpy(&bufferCalibration[90], &inclinacao.vector, MAX_BYTE_VECTOR_XYZ);

		// -> 64 bytes a struct

		/////////////////////CRC16/////////////////////

		crc16_calc.crc16 = crc16c(&bufferCalibration[1], 113);

		memcpy(&bufferCalibration[114], crc16_calc.crc16_bytes, 2);

		/////////////////////Send Serial///////////////

		calSendData( bufferCalibration, 116);

		state_cal = CAL_WAIT;

	break;

	case CAL_READ_STATUS:

		//faz a leitura do status da calibracao

		Memoria_ReadData(&memoria);

		/////////////////////////////////////////

		memcpy(&bufferCalibration[2], &memoria.Calibrado, 1);

		/////////////////////CRC16/////////////////////

		crc16_calc.crc16 = crc16c(&bufferCalibration[1], 2);

		memcpy(&bufferCalibration[3], crc16_calc.crc16_bytes, 2);

		/////////////////////Send Serial///////////////

		calSendData( bufferCalibration, 5);

		state_cal = CAL_WAIT;

	break;

	case CAL_WRITE_MATRIX_XYZ:

		/////////////////////////////////////////
#ifdef SHORT_CAL
		crc16_calc.crc16 = crc16c(&bufferCalibration[1], curCalIndex - 3);
		memcpy(crc16_recv.crc16_bytes, &bufferCalibration[curCalIndex - 2], 2);
#else
		crc16_calc.crc16 = crc16c(&bufferCalibration[1], 73);
		memcpy(crc16_recv.crc16_bytes, &bufferCalibration[74], 2);
#endif

		if (crc16_calc.crc16 == crc16_recv.crc16) {

			matrixCal = 1;

			/////////////////////////////////////////

			memcpy(&inclinacao.matrix.BytesMatrixGanho, &bufferCalibration[2], MAX_BYTE_MATRIX_XYZ);

			/////////////////////CRC16/////////////////////

			crc16_calc.crc16 = crc16c(&bufferCalibration[1], 1);

			memcpy(&bufferCalibration[2], crc16_calc.crc16_bytes, 2);

			/////////////////////Send Serial///////////////

			calSendData( bufferCalibration, 4);
			//LPUART_DRV_SendData(FSL_LPUART_CAL, bufferCalibration, 4);
		}

		state_cal = CAL_WAIT;

		break;

	case CAL_WRITE_VECTOR_XYZ:

		/////////////////////////////////////////
#ifdef SHORT_CAL
		crc16_calc.crc16 = crc16c(&bufferCalibration[1], curCalIndex - 3);
		memcpy(crc16_recv.crc16_bytes, &bufferCalibration[curCalIndex - 2], 2);
#else
		crc16_calc.crc16 = crc16c(&bufferCalibration[1], 25);
		memcpy(crc16_recv.crc16_bytes, &bufferCalibration[26], 2);
#endif

		if (crc16_calc.crc16 == crc16_recv.crc16) {

			vectorCal = 1;

			/////////////////////////////////////////

			memcpy(&inclinacao.vector.BytesVectorOffset, &bufferCalibration[2], MAX_BYTE_VECTOR_XYZ);

			/////////////////////CRC16/////////////////////

			crc16_calc.crc16 = crc16c(&bufferCalibration[1], 1);

			memcpy(&bufferCalibration[2], crc16_calc.crc16_bytes, 2);

			/////////////////////Send Serial///////////////

			calSendData( bufferCalibration, 4);

		}

		state_cal = CAL_WAIT;

		break;

	case CAL_WRITE_CALIBRATION:

		/////////////////////////////////////////
#ifdef SHORT_CAL
		crc16_calc.crc16 = crc16c(&bufferCalibration[1], curCalIndex - 3);
		memcpy(crc16_recv.crc16_bytes, &bufferCalibration[curCalIndex - 2], 2);
#else
		crc16_calc.crc16 = crc16c(&bufferCalibration[1], 51);
		memcpy(crc16_recv.crc16_bytes, &bufferCalibration[52], 2);
#endif

		if (crc16_calc.crc16 == crc16_recv.crc16) {

			/////////////////////////////////////////

			//   AA  +  Code  +  bufIndex  +  matrix  +  CRC16
			//1 byte + 1 byte + 2 bytes + 48 bytes + 2 bytes = 54bytes

			memcpy(&bufIndex, &bufferCalibration[2], 2);

			/*if(bufIndex == 0){

				memoria.CorOrientador = 0;

				memset(&memoria.AnguloOffset, 0, 6);
				memset(&memoria.IncOffset, 0, 6);

				memoria.FlagZerouTF = 0;
				memoria.OrientadorStatus = 0;
				memoria.Calibrado = 0;

				memset(&inclinacao.matrix, 0, MAX_BYTE_MATRIX_XYZ);
				memset(&inclinacao.vector, 0, MAX_BYTE_VECTOR_XYZ);

				memset(&calibracao.BytesCalibracao[0], 0, MAX_BYTE_CALIBRACAO);

				/////////////////////////////////////////
				LOGGER_ApagarCalibracao();

				FlashSaveAll(&memoria);
			}*/
			memcpy(&calibracao.pontos[bufIndex], &bufferCalibration[4], 48);

			/////////////////////CRC16/////////////////////

			crc16_calc.crc16 = crc16c(&bufferCalibration[1], 1);

			memcpy(&bufferCalibration[2], crc16_calc.crc16_bytes, 2);

			/////////////////////Send Serial///////////////

			calSendData( bufferCalibration, 4);

		}

		state_cal = CAL_WAIT;

		break;

	case CAL_WRITE_FINAL_CRC:

		/////////////////////////////////////////

		crc16_calc.crc16 = crc16c(&bufferCalibration[1], 3);

		memcpy(crc16_recv.crc16_bytes, &bufferCalibration[4], 2);

		if (crc16_calc.crc16 == crc16_recv.crc16) {

			/////////////////////////////////////////

			//   AA  +  Code  +  CRC16(data)  +  CRC16
			//1 byte + 1 byte + 2 bytes + 2 bytes = 6bytes
			crc16_calc.crc16 = Cal_CalculaCRC16Calibracao();

			memcpy(crc16_recv.crc16_bytes, &bufferCalibration[2], 2);
			//crc16_calc.crc16 = crc16_recv.crc16;
			if (crc16_calc.crc16 == crc16_recv.crc16) {

				memcpy(&memoria.CRC, &bufferCalibration[2], 2);
				//crc16_calc.crc16 = Cal_CalculaCRC16Calibracao();
				//memcpy(&memoria.CRC, crc16_calc.crc16_bytes, 2);
				memset(&bufferCalibration[2],1,1);
			} else {

				memset(&bufferCalibration[2],0,1);
				//memset(&bufferCalibration[2],1,1);
			}

			/////////////////////CRC16/////////////////////

			crc16_calc.crc16 = crc16c(&bufferCalibration[1], 2);

			memcpy(&bufferCalibration[3], crc16_calc.crc16_bytes, 2);

			/////////////////////Send Serial///////////////

			calSendData( bufferCalibration, 5);

		}

		state_cal = CAL_WAIT;

		break;
	case CAL_RESET:

		//apaga a memoria de calibracao

		/////////////////////////////////////////

		memoria.CorOrientador = 0;

		memset(&memoria.AnguloOffset, 0, 6);
		memset(&memoria.IncOffset, 0, 6);

		memoria.FlagZerouTF = 0;
		memoria.OrientadorStatus = 0;
		memoria.Calibrado = 0;

		memset(&inclinacao.matrix, 0, MAX_BYTE_MATRIX_XYZ);
		memset(&inclinacao.vector, 0, MAX_BYTE_VECTOR_XYZ);

		memset(&calibracao.BytesCalibracao[0], 0, MAX_BYTE_CALIBRACAO);

		/////////////////////////////////////////
		LOGGER_ApagarCalibracao();

		FlashSaveAll(&memoria);

		/////////////////////CRC16/////////////////////

		crc16_calc.crc16 = crc16c(&bufferCalibration[1], 1);

		memcpy(&bufferCalibration[2], crc16_calc.crc16_bytes, 2);

		/////////////////////Send Serial///////////////

		calSendData( bufferCalibration, 4);

		state_cal = CAL_WAIT;

		break;

	case CAL_FINISH:

		//termina o processo de calibracao

		if (matrixCal == 1 && vectorCal == 1) { //se j� calibrou o vector

			memoria.Calibrado = 1;
			FlashSaveAll(&memoria);

			LOGGER_AtualizaCalibracao();

			/////////////////////CRC16/////////////////////

			/*crc16_calc.crc16 = Flash_CalculaCRC16Calibracao(); //se calibrou corretamente retorno o CRC16 do que foi gravado na flash

			memcpy(&bufferCalibration[2], crc16_calc.crc16_bytes, 2);
			memset(&memoria.CRC, 0, 2);
			memcpy(&memoria.CRC, crc16_calc.crc16_bytes, 2);*/

		} else {

			/////////////////////CRC16/////////////////////

			crc16_calc.crc16 = crc16c(&bufferCalibration[1], 1); //caso n�o esteja calibrando retorna apenas o CRC16 do comando

			memcpy(&bufferCalibration[2], crc16_calc.crc16_bytes, 2);
		}

		/////////////////////Send Serial///////////////

		calSendData( bufferCalibration, 4);

		state_cal = CAL_WAIT;

		//Terminou calibracao

		CalClearBuffer();

		startCal = 0;

		bufIndex = 0;

		matrixCal = 0;
		vectorCal = 0;

		break;

	case CAL_WRITE_FLASH:

			//termina o processo de calibracao

			if (matrixCal == 1 && vectorCal == 1) { //se ja calibrou o vector

				memoria.Calibrado = 1;
				/////***********/////FlashSaveAll(&memoria);

				LOGGER_AtualizaCalibracao();

				/////////////////////CRC16/////////////////////

				/*crc16_calc.crc16 = Flash_CalculaCRC16Calibracao(); //se calibrou corretamente retorno o CRC16 do que foi gravado na flash

				memcpy(&bufferCalibration[2], crc16_calc.crc16_bytes, 2);
				memset(&memoria.CRC, 0, 2);
				memcpy(&memoria.CRC, crc16_calc.crc16_bytes, 2);*/

			} else {

				/////////////////////CRC16/////////////////////

				crc16_calc.crc16 = crc16c(&bufferCalibration[1], 1); //caso n�o esteja calibrando retorna apenas o CRC16 do comando

				memcpy(&bufferCalibration[2], crc16_calc.crc16_bytes, 2);
			}

			/////////////////////Send Serial///////////////

			calSendData( bufferCalibration, 4);

			state_cal = CAL_WAIT;

			//Terminou calibracao

			CalClearBuffer();

			matrixCal = 0;
			vectorCal = 0;
			bufIndex = 0;
			break;

	case CAL_OT_VER:{

		//memcpy(&bufferCalibration[2],"1.6",3);
		bufferCalibration[2] = 2;
		bufferCalibration[3] = 0;

		crc16_calc.crc16 = crc16c(&bufferCalibration[1], 3);
		memcpy(&bufferCalibration[4], crc16_calc.crc16_bytes, 2);
		calSendData( bufferCalibration, 6);
		state_cal = CAL_WAIT;
	}break;

	case CAL_READ_CRC:{

		unsigned short crcFlash = Flash_CalculaCRC16Calibracao();
		memcpy(&bufferCalibration[2], &crcFlash, 2);
		crc16_calc.crc16 = crc16c(&bufferCalibration[1], 3);
		memcpy(&bufferCalibration[4], crc16_calc.crc16_bytes, 2);
		calSendData( bufferCalibration, 6);
		state_cal = CAL_WAIT;
	}break;
	default:

		//caso seja um comando invalido termina a calibracao forcadamente

		startCal = 0; //Forca terminar calibracao

		matrixCal = 0;
		vectorCal = 0;

		state_cal = CAL_WAIT;

		break;
	}

	k_msleep(200);
}

/**
 *
 */
state_cal_receive_t CalReceive() {

	return state_cal_receive;
}

/**
 *
 */
void CalClearBuffer() {

	memset(bufferCalRx, 0, MAX_RX_BUFFER); //Limpa o buffer

	calIndex = 0; //zera o bufIndex do buffer

	state_cal_receive = CAL_RECEIVE_NONE; //seta o flag para iniciar novo recebimento
}

/**
 *
 */
unsigned short crc16c(const unsigned char* data_p, unsigned char length) {

	unsigned char x;

	unsigned short crc = 0xFFFF;

	while (length--) {

		x = crc >> 8 ^ *data_p++;

		x ^= x >> 4;

		crc = (crc << 8) ^ ((unsigned short) (x << 12)) ^ ((unsigned short) (x << 5)) ^ ((unsigned short) x);
	}

	return crc;
}

uint8_t GetCalCRCFlash(){

	CRC16_u tempCRC;

	/////***********/////Flash_ReadCRC();

	tempCRC.crc16 = Flash_CalculaCRC16Calibracao();
	
	if(((memoria.CRC[0] == tempCRC.crc16_bytes[0])&&(memoria.CRC[1] == tempCRC.crc16_bytes[1]))||forceCalFlag){

		return true;

	} else{

		return false;

	}

}

/**
 *
 */
unsigned short Cal_CalculaCRC16Calibracao() {

	unsigned char BufferAux[96];

	unsigned char x;

	unsigned short crc = 0xFFFF;

	int j = 0;

	int length = 17280;

	memset(BufferAux, 0, 96);
	memcpy(&BufferAux[0], &(inclinacao.matrix), MAX_BYTE_MATRIX_XYZ);
	memcpy(&BufferAux[72], &(inclinacao.vector), MAX_BYTE_VECTOR_XYZ);

	while (length--) {

		if (j < 96){
			x = crc >> 8 ^ BufferAux[(j++)];
		} else {
			x = crc >> 8 ^ calibracao.BytesCalibracao[(j++)-96];
		}

		x ^= x >> 4;

		crc = (crc << 8) ^ ((unsigned short) (x << 12)) ^ ((unsigned short) (x << 5)) ^ ((unsigned short) x);
	}

	return crc;
}

void calSendData(unsigned char *data, unsigned char size) {
	
	for (int i = 0; i < size; i++) {
		uart_poll_out(cdc1_dev, data[i]);
	}
}

void uartCalInit(void){

}

void uartCalDeInit(void){

}

void uartCalCb(const struct device *dev, void *user_data){

	uint32_t bytes_read;

	if (!uart_irq_update(dev)) {
		return;
	}

	if (!uart_irq_rx_ready(dev)) {
		return;
	}

	if (state_cal_receive == CAL_RECEIVE_NONE && calIndex == 0) {

   		state_cal_receive = CAL_RECEIVING;
   	}

   	if (state_cal_receive == CAL_RECEIVING) {

		//while (uart_fifo_read(dev, &calDadoRx, 1) == 1) {
			while ((bytes_read = uart_fifo_read(dev,
					(uint8_t *)bufferCalRx+calIndex,
					sizeof(bufferCalRx)-calIndex))) {
					calIndex += bytes_read;

			if (calIndex < MAX_RX_BUFFER) {
				//bufferCalRx[calIndex++] = calDadoRx;
			} else {
				state_cal_receive = CAL_RECEIVED_OVERFLOW;
				calIndex = 0;
				k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
			}
		}
   	}

}

void uartCmdCb(const struct device *dev, void *user_data){

	uint32_t bytes_read;

	if (!uart_irq_update(dev)) {
		return;
	}

	if (!uart_irq_rx_ready(dev)) {
		return;
	}

	while ((bytes_read = uart_fifo_read(dev,
			(uint8_t *)bufferCmdRx+cmdIndex,
			sizeof(bufferCmdRx)-cmdIndex))) {

			cmdIndex += bytes_read;

		if (cmdIndex >= MAX_RX_BUFFER){

			memset(bufferCmdRx, 0, sizeof(bufferCmdRx));
			cmdIndex = 0;
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

		}else if(bufferCmdRx[cmdIndex - 1] == '\r'){

			parseCmd = true;
			uart_irq_rx_disable(cdc1_dev);
		}

	}
   	

}

bool readXYZ(void){

	uint32_t enterTick = sys_clock_tick_get_32();
	uint32_t cTick = enterTick;

	newAccData = false;
	while(!newAccData && ((cTick - enterTick) < 10000)){
		//Waiting for new package to be received via BLE
		cTick = sys_clock_tick_get_32();
	};
	
	return 0;
}

bool inc_GRAV_FIFO_raw(double* x, double* y, double* z){

	x[0] = bleSensData.xRaw;
	y[0] = bleSensData.yRaw;
	z[0] = bleSensData.zRaw;

	return 0;
}

double inc_GRAV_X(void){

	return bleSensData.xCal;
}

double inc_GRAV_Y(void){

	return bleSensData.yCal;
}

double inc_GRAV_Z(void){

	return bleSensData.zCal;
}

void BleOff(void){
	
}

void LOGGER_AtualizaCalibracao(void) {

	unsigned char bufferWrite[1] = { 0x01 };
	CRC16_u tempCRC;
	mb85rs1WriteData(0x04, bufferWrite, 0x01); 

	mb85rs1WriteData(0x05, inclinacao.matrix.BytesMatrixGanho, MAX_BYTE_MATRIX_XYZ); 

	mb85rs1WriteData(0x05 + MAX_BYTE_MATRIX_XYZ, inclinacao.vector.BytesVectorOffset, MAX_BYTE_VECTOR_XYZ); 

	mb85rs1WriteData(0x05 + MAX_BYTE_MATRIX_XYZ + MAX_BYTE_VECTOR_XYZ, calibracao.BytesCalibracao, MAX_BYTE_CALIBRACAO); 

	mb85rs1WriteData(0x05 + MAX_BYTE_MATRIX_XYZ + MAX_BYTE_VECTOR_XYZ + MAX_BYTE_CALIBRACAO, memoria.CRC, 2);

	tempCRC.crc16 = LOGGER_CalculaCRC16Calibracao();
	if (strstr(tempCRC.crc16_bytes, memoria.CRC)){

		memoria.Calibrado = 1;

	} else {

		memoria.Calibrado = 0;
	}
}

void LOGGER_ApagarCalibracao(void) {

	mb85rs1WriteData(0x04, 0, 0x01); // Grava na memoria.

	mb85rs1WriteData(0x05, 0, MAX_BYTE_MATRIX_XYZ); // Busca na memoria.

	mb85rs1WriteData(0x05 + MAX_BYTE_MATRIX_XYZ, 0, MAX_BYTE_VECTOR_XYZ); // Busca na memoria.

	mb85rs1WriteData(0x05 + MAX_BYTE_MATRIX_XYZ + MAX_BYTE_VECTOR_XYZ, 0, MAX_BYTE_CALIBRACAO); // Busca na memoria.

	mb85rs1WriteData(0x05 + MAX_BYTE_MATRIX_XYZ + MAX_BYTE_VECTOR_XYZ + MAX_BYTE_CALIBRACAO, 0, 2);
}

unsigned short LOGGER_CalculaCRC16Calibracao(void) {

	unsigned char x;

	unsigned short crc = 0xFFFF;

	int j = 0;

	int length = 17280;
	unsigned char buffer[1];
	while (length--) {

		mb85rs1ReadData(0x05 + j, buffer, 1);
		j++;
		x = crc >> 8 ^ buffer[0];

		x ^= x >> 4;

		crc = (crc << 8) ^ ((unsigned short) (x << 12)) ^ ((unsigned short) (x << 5)) ^ ((unsigned short) x);
	}

	return crc;
}