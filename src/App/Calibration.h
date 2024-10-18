#ifndef APP_CALIBRATION_H_
#define APP_CALIBRATION_H_

#include "../main.h"

///////////////////defines///////////////////////////
#define CAL_UART UART1

#define MAX_RX_BUFFER			120
#define MAX_BUFFER				120

#define MAX_ELEMENTS_MATRIX_XYZ		9		//9 variaveis do tipo double

#define MAX_BYTE_MATRIX_XYZ			(MAX_ELEMENTS_MATRIX_XYZ * sizeof(double))		//72 bytes (MAX_ELEMENTS_MATRIX * sizeof(float))

#define MAX_ELEMENTS_VECTOR_XYZ		3		//3 variaveis do tipo double

#define MAX_BYTE_VECTOR_XYZ			(MAX_ELEMENTS_VECTOR_XYZ * sizeof(double))		//24 bytes (MAX_ELEMENTS_VECTOR * sizeof(float))

#define MAX_ELEMENTS_CALIBRATION		358

#define MAX_BYTE_ELEMENT_CALIBRATION	48

#define MAX_BYTE_CALIBRACAO			(MAX_ELEMENTS_CALIBRATION * MAX_BYTE_ELEMENT_CALIBRATION)		//358 elementos de 48 bytes cada -> 17184

#define CAL_FLAG_ADDR 		0x04
#define INC_MATRIX_ADDR 	CAL_FLAG_ADDR + 0x01
#define INC_VECTOR_ADDR 	INC_MATRIX_ADDR + MAX_BYTE_MATRIX_XYZ
#define CAL_BYTES_ADDR		INC_VECTOR_ADDR + MAX_BYTE_VECTOR_XYZ
#define CRC_BYTES_ADDR		CAL_BYTES_ADDR + MAX_BYTE_CALIBRACAO
#define CAL_FULL_SIZE		MAX_BYTE_MATRIX_XYZ + MAX_BYTE_VECTOR_XYZ + MAX_BYTE_CALIBRACAO

//um pacote de comando sempre come�a com 0xAA e sempre termina com 0x0A

#define CAL_RECV_PACKAGE				0xAA
#define CAL_SENT_PACKAGE				0xBB

#define CAL_FINISH_PACKAGE				0x0A

//comando START 0xAA + 0xBA
//retorno 0xBB + 0xBA + CRC16
#define CAL_CMD_START					0xBA

//comando RESET 0xAA + 0xBB
//retorno 0xBB + 0xBB + CRC16
#define CAL_CMD_RESET					0xBB

//comando FINISH 0xAA + 0xBC
//retorno 0xBB + 0xBC + CRC16
#define CAL_CMD_FINISH					0xBC

//comando READ RAW 0xAA + 0CA
//retorno 0xBB + 0xCA + [24 bytes{3 variaveis de 8 bytes}] + CRC16
#define CAL_CMD_READ_RAW				0xCA

//comando READ MEMORY CONFIG 0xAA + 0xCB
//retorno 0xBB + 0xCB + [52 bytes{ver o typedef 'memoria_flash_orientador_t'}] + CRC16
#define CAL_CMD_READ_MEMORY_CONFIG		0xCB

//comando READ CAL 0xAA + 0xCC
//retorno 0xBB + 0xCC + [24 bytes{3 variaveis de 8 bytes}] + CRC16
#define CAL_CMD_READ_CAL				0xCC

//comando READ STATUS 0xAA + 0xCD
//retorno 0xBB + 0xCD + [1 bytes{0 descalibrado 1 calibrado}]+ CRC16
#define CAL_CMD_READ_STATUS				0xCD

//comando READ OT VERSION 0xAA + 0xCE + CRC16
//retorno 0xBB + 0xEE + [2 bytes versao] + CRC16
#define CAL_CMD_OT_VER					0xCE

//comando READ CRC FLASH 0xAA + 0xCF + CRC16
//retorno 0xBB + 0xEF + [2 bytes CRC] + CRC16
#define CAL_CMD_READ_CRC				0xCF

//comando WRITE MATRIX 0xAA + 0xEA + [72 bytes da matriz de calibracao{9 variaveis de 8 bytes}]
//retorno 0xBB + 0xEA + CRC16
#define CAL_CMD_WRITE_MATRIX_XYZ		0xEA

//comando WRITE VECTOR 0xAA + 0xEB + [24 bytes da vetor de calibracao{3 variaveis de 8 bytes}]
//retorno 0xBB + 0xEB + CRC16
#define CAL_CMD_WRITE_VECTOR_XYZ		0xEB

//comando WRITE MATRIX 0xAA + 0xEC + [1 byte do index] + [48 bytes da matriz de calibracao{4 variaveis de 8 bytes}]
//retorno 0xBB + 0xEC + CRC16
#define CAL_CMD_WRITE_CALIBRATION		0xEC

//comando WRITE FINAL CRC 0xAA + 0xED + [4 bytes CRC16 da matriz completa de calibracao] + CRC16
//retorno 0xBB + 0xED + [1 bytes{0 CRC16 falha 1 CRC16 ok}] + CRC16
#define CAL_CMD_WRITE_FINAL_CRC			0xED

//comando WRITE CALIBRATION ON FLASH 0xAA + 0xEF + CRC16
//retorno 0xBB + 0xEF + CRC16
#define CAL_CMD_WRITE_FLASH				0xEF

#define WAIT_A_USEC						1
#define WAIT_A_MSEC						1000
#define WAIT_A_SEC						1000000
///////////////type def's/////////////////////

typedef enum {
	CAL_RECEIVE_NONE, CAL_RECEIVING, CAL_RECEIVED_OVERFLOW
}state_cal_receive_t;

typedef enum {
	CAL_NONE,
	CAL_WAIT,
	CAL_START,
	CAL_READ_RAW,
	CAL_READ_CAL,
	CAL_READ_MEMORY_CONFIG,
	CAL_READ_STATUS,
	CAL_WRITE_MATRIX_XYZ,
	CAL_WRITE_VECTOR_XYZ,
	CAL_WRITE_CALIBRATION,
	CAL_WRITE_FINAL_CRC,
	CAL_RESET,
	CAL_FINISH,
	CAL_OT_VER,
	CAL_READ_CRC,
	CAL_WRITE_FLASH
}state_cal_t;

typedef union union_crc16 {

	unsigned short crc16;

	unsigned char crc16_bytes[2];

} CRC16_u;
///////////////variaveis externas/////////////////////

extern char startCal;

extern unsigned char calIndex;

extern unsigned char bufferCalRx[];
extern unsigned char bufferCalibration[];

extern state_cal_receive_t state_cal_receive;

extern unsigned char CalIsOff;
extern unsigned char cmdIndex;
extern unsigned char bufferCmdRx[MAX_RX_BUFFER];
extern bool parseCmd;

///////////////metodos publicos/////////////////////

void Calibracao();

void CalClearBuffer();

void CalOff();

void CalOn();

uint8_t GetCalCRCFlash();

unsigned short Cal_CalculaCRC16Calibracao();

void calSendData(unsigned char *data, unsigned char size);
void uartCalInit(void);
void uartCalDeInit(void);
void uartCalCb(const struct device *dev, void *user_data);
void uartCmdCb(const struct device *dev, void *user_data);
void BleOff(void);
bool readXYZ(void);
bool inc_GRAV_FIFO_raw(double* x, double* y, double* z);
double inc_GRAV_X(void);
double inc_GRAV_Y(void);
double inc_GRAV_Z(void);

/*****From MEMORIA.H*******/
#define LAUNCH_CMD_SIZE           0x100

typedef struct {

	unsigned char CorOrientador;    // 0 = Azul     1 = Verde.
	unsigned char AnguloOffset[6];  // Angulo para offset do Zerar TF.
	unsigned char IncOffset[6];  	// Inclinacao para offset do Zerar TF.
	unsigned char FlagZerouTF; 		// Quando executar a funcao zerar TF, esse flag vai pra 1.
	unsigned char OrientadorStatus;
	unsigned char Calibrado;
	unsigned char CRC[2];

} memoria_flash_orientador_t;

//////////////////////inclinacao

typedef union {

	double MatrixGanho[MAX_ELEMENTS_MATRIX_XYZ];

	char BytesMatrixGanho[MAX_BYTE_MATRIX_XYZ];

} matrix_inclinacao;

typedef union {

	double VectorOffset[MAX_ELEMENTS_VECTOR_XYZ];

	char BytesVectorOffset[MAX_BYTE_VECTOR_XYZ];

} vetor_inclinacao;

typedef struct {

	matrix_inclinacao matrix;
	vetor_inclinacao vector;

} valores_inclinacao;

//////////////////////calibracao

typedef union {

	double MatrixGanho[2][3];

	char BytesMatrixGanho[MAX_BYTE_ELEMENT_CALIBRATION]; //6 variaveis de 8 bytes

} calibracao_matrix_vector;

typedef union {

	calibracao_matrix_vector pontos[MAX_ELEMENTS_CALIBRATION];

	char BytesCalibracao[MAX_BYTE_CALIBRACAO]; //

} valores_calibracao;

void Memoria_SaveData(memoria_flash_orientador_t *memoria);
void FlashSaveAll(memoria_flash_orientador_t* memoria);
void Memoria_ReadData(memoria_flash_orientador_t *memoria);
void FlashReadAll(memoria_flash_orientador_t* memoria);
unsigned short Flash_CalculaCRC16Calibracao();
void Flash_ReadCRC();
void initFlash(void);

extern memoria_flash_orientador_t memoria;

extern valores_calibracao calibracao;

extern valores_inclinacao inclinacao;

extern CRC16_u crc16_calc;
/**************************/

/*****From LOGGER.H*******/
void LOGGER_AtualizaCalibracao(void);
void LOGGER_ApagarCalibracao(void);
unsigned short LOGGER_CalculaCRC16Calibracao(void);
/**************************/
#endif /* APP_CALIBRATION_H_ */
