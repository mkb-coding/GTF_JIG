#include "Calibration.h"

#define DATA_ADD 0x3f800
char *DadosFlash = (char*)DATA_ADD;
#define CAL_ADD 0x3b000
char *DadosCal = (char*)CAL_ADD;

memoria_flash_orientador_t memoria;

valores_calibracao calibracao;

valores_inclinacao inclinacao;

/**
 *
 */
void Memoria_SaveData(memoria_flash_orientador_t *memoria) {

	unsigned char BufferAux[1024];
	uint32_t pflashSectorSize = 0;

	
	memset(BufferAux, 0, 1024);

	BufferAux[0] = memoria->CorOrientador;
	BufferAux[1] = memoria->AnguloOffset[0];
	BufferAux[2] = memoria->AnguloOffset[1];
	BufferAux[3] = memoria->AnguloOffset[2];
	BufferAux[4] = memoria->AnguloOffset[3];
	BufferAux[5] = memoria->AnguloOffset[4];
	BufferAux[6] = memoria->AnguloOffset[5];
	BufferAux[7] = memoria->IncOffset[0];
	BufferAux[8] = memoria->IncOffset[1];
	BufferAux[9] = memoria->IncOffset[2];
	BufferAux[10] = memoria->IncOffset[3];
	BufferAux[11] = memoria->IncOffset[4];
	BufferAux[12] = memoria->IncOffset[5];
	BufferAux[13] = memoria->FlagZerouTF;
	BufferAux[14] = memoria->OrientadorStatus;
	BufferAux[15] = memoria->Calibrado;

/*
	__disable_irq();
	FLASH_Erase(&s_flashDriver, DATA_ADD, pflashSectorSize, kFTFx_ApiEraseKey);
	FLASH_Program(&s_flashDriver, DATA_ADD, (uint8_t *)BufferAux, sizeof(BufferAux));
	__enable_irq();
*/
}

/**
 *
 */
void FlashSaveAll(memoria_flash_orientador_t* memoria) {

	//17296

	unsigned char BufferAux[1024];
	uint32_t pflashSectorSize = 0;

	memset(BufferAux, 0, 1024);

	BufferAux[0] = memoria->CorOrientador;
	BufferAux[1] = memoria->AnguloOffset[0];
	BufferAux[2] = memoria->AnguloOffset[1];
	BufferAux[3] = memoria->AnguloOffset[2];
	BufferAux[4] = memoria->AnguloOffset[3];
	BufferAux[5] = memoria->AnguloOffset[4];
	BufferAux[6] = memoria->AnguloOffset[5];
	BufferAux[7] = memoria->IncOffset[0];
	BufferAux[8] = memoria->IncOffset[1];
	BufferAux[9] = memoria->IncOffset[2];
	BufferAux[10] = memoria->IncOffset[3];
	BufferAux[11] = memoria->IncOffset[4];
	BufferAux[12] = memoria->IncOffset[5];
	BufferAux[13] = memoria->FlagZerouTF;
	BufferAux[14] = memoria->OrientadorStatus;
	BufferAux[15] = memoria->Calibrado;
/*
	__disable_irq();
	//FLASH_Erase(&s_flashDriver, DATA_ADD, 10 * pflashSectorSize, kFTFx_ApiEraseKey);
	FLASH_Erase(&s_flashDriver, CAL_ADD, 10 * pflashSectorSize, kFTFx_ApiEraseKey);
	FLASH_Program(&s_flashDriver, DATA_ADD, (uint8_t *)BufferAux, sizeof(BufferAux));
	__enable_irq();
*/
	//////////////

	memset(BufferAux, 0, 1024);
	memcpy(&BufferAux[0], &(inclinacao.matrix), MAX_BYTE_MATRIX_XYZ);
	memcpy(&BufferAux[72], &(inclinacao.vector), MAX_BYTE_VECTOR_XYZ);
	memcpy(&BufferAux[96], &calibracao.BytesCalibracao[0], 928);

	//////////////
/*
	__disable_irq();
	FLASH_Program(&s_flashDriver, CAL_ADD, (uint8_t *)BufferAux, sizeof(BufferAux));
	__enable_irq();
	//////////////

	int i = 0;

	int j = 1;

	while(i < 15){

		memset(BufferAux, 0, 1024);

		memcpy(&BufferAux[0], &calibracao.BytesCalibracao[928 + (i * 1024)], 1024);

		//////////////
		__disable_irq();
		//FLASH_Erase(&s_flashDriver, CAL_ADD + (j * 1024), pflashSectorSize, kFTFx_ApiEraseKey);
		FLASH_Program(&s_flashDriver, CAL_ADD + (j * 1024), (uint8_t *)BufferAux, sizeof(BufferAux));
		__enable_irq();
		//////////////

		i++;

		j++;
	}

	memset(BufferAux, 0, 1024);

	memcpy(&BufferAux[0], &calibracao.BytesCalibracao[928 + (i * 1024)], 896);

	//////////////
	__disable_irq();
	//FLASH_Erase(&s_flashDriver, CAL_ADD + (j * 1024), pflashSectorSize, kFTFx_ApiEraseKey);
	FLASH_Program(&s_flashDriver, CAL_ADD + (j * 1024), (uint8_t *)BufferAux, 896);
	__enable_irq();
*/	//////////////

	memset(BufferAux, 0, 1024);
	memcpy(&BufferAux[0], &memoria->CRC[0], 2);
/*
	__disable_irq();
	FLASH_Program(&s_flashDriver, CAL_ADD + (j * 1024) + 896, (uint8_t *)BufferAux, 4);
	__enable_irq();
	WDOG_Refresh(WDOG);
*/
}

/**
 *
 */
void Memoria_ReadData(memoria_flash_orientador_t *memoria) {

	unsigned char i = 0, j = 0;

	memoria->CorOrientador = DadosFlash[j++];

	for (i = 0; i < 6; i++)
		memoria->AnguloOffset[i] = DadosFlash[j++];

	for (i = 0; i < 6; i++)
		memoria->IncOffset[i] = DadosFlash[j++];

	memoria->FlagZerouTF = DadosFlash[j++];

	j++; //memoria->OrientadorStatus nao carrega essa posicao, incrementa o contador

	memoria->Calibrado = DadosFlash[j++];

}

void FlashReadAll(memoria_flash_orientador_t* memoria) {

	uint16_t i = 0, j = 0;

	memoria->CorOrientador = DadosFlash[j++];

	for (i = 0; i < 6; i++)
		memoria->AnguloOffset[i] = DadosFlash[j++];

	for (i = 0; i < 6; i++)
		memoria->IncOffset[i] = DadosFlash[j++];

	memoria->FlagZerouTF = DadosFlash[j++];

	j++; //memoria->OrientadorStatus nao carrega essa posicao, incrementa o contador

	memoria->Calibrado = DadosFlash[j++];

	j = 0;

	memcpy(&inclinacao.matrix, &DadosCal[j], MAX_BYTE_MATRIX_XYZ);

	j += MAX_BYTE_MATRIX_XYZ;

	memcpy(&inclinacao.vector, &DadosCal[j], MAX_BYTE_VECTOR_XYZ);

	j += MAX_BYTE_VECTOR_XYZ;

	memcpy(&calibracao.BytesCalibracao, &DadosCal[j], MAX_BYTE_CALIBRACAO);
	Flash_ReadCRC();
}

/**
 *
 */
unsigned short Flash_CalculaCRC16Calibracao() {

	unsigned char x;
	unsigned short crc = 0xFFFF;
	//int j = 0;
	int length = 17280;
	uint8_t rdBuf[1] = {0};
	uint32_t addr = 0x05;
	while (length--) {

		mb85rs1ReadData(addr, rdBuf, 1);
		addr++;
		//x = crc >> 8 ^ DadosCal[j++];
		x = crc >> 8 ^ rdBuf[0];

		x ^= x >> 4;

		crc = (crc << 8) ^ ((unsigned short) (x << 12)) ^ ((unsigned short) (x << 5)) ^ ((unsigned short) x);
	}

	return crc;
}
void Flash_ReadCRC(){

	memset(memoria.CRC, 0, 2);
	memcpy(&memoria.CRC[0], &DadosCal[17280],2);
}

void initFlash(void){

	/*uint32_t pflashBlockBase = 0;
	uint32_t pflashTotalSize = 0;
	uint32_t pflashSectorSize = 0;*/

}
