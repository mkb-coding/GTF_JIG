#include "MB85RS1.h"

struct device *devSpiAccMem;
struct spi_config devSpiAccMemCfg = {
		.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB /*| SPI_MODE_CPOL | SPI_MODE_CPHA*/,
		.frequency = 4000000,
		.slave = 0,
};

struct gpioPinSettings {
	const char * const port;
	const uint8_t number;
	uint32_t mode;
};

static const struct gpioPinSettings gpioPins[] = {

	{"GPIO_1",
	 DT_GPIO_PIN(DT_NODELABEL(memcsn), gpios),
	 GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH},
	
};

static const struct device *gpioDevs[ARRAY_SIZE(gpioPins)];

uint8_t mb85rs1TransceiveCmd(uint32_t wLen, uint8_t* wBuf,
                             uint32_t wLen1, uint8_t* wBuf1,
                             uint32_t rLen, uint8_t* rBuf){

	SRAM_CS_ENABLE;

	struct spi_buf txbufs[] = {
		{
			.buf = wBuf,
			.len = wLen,
		},
        {
			.buf = wBuf1,
			.len = wLen1,
		}};

	struct spi_buf_set tx = {
		.buffers = txbufs,
		.count = 2,
	};

    tx.count = wLen1? 2 : 1;

	struct spi_buf rxbufs[] = {
		{
			.buf = rBuf,
		 	.len = rLen,
		}};

	struct spi_buf_set rx = {
		.buffers = rxbufs,
		.count = 1,
	};
	
	if(wLen && (wBuf != NULL)){
	    spi_transceive(devSpiAccMem, &devSpiAccMemCfg, &tx, NULL);
    }

    if(rLen && (rBuf != NULL)){
	    spi_transceive(devSpiAccMem, &devSpiAccMemCfg, NULL, &rx);
    }
	
	SRAM_CS_DISABLE;

	return 0;
}
uint8_t mb85rs1Init(void){

    uint8_t srVal[1] = {MB85RS1_WPEN_DIS | MB85RS1_BPNONE};
    uint32_t pBuf = 0;
    /*SCL3300_CS_DISABLE;
	SCL3300_PW_ENABLE;
    mb85rs1SetSleep(0);
    mb85rs1ReadSr(srVal);*/

    int err = 0;
	gpio_flags_t flags = 0;
	
	for (size_t i = 0; i < ARRAY_SIZE(gpioPins); i++) {
		gpioDevs[i] = device_get_binding(gpioPins[i].port);
		if (!gpioDevs[i]) {
			//LOG_ERR("Cannot bind gpio device");
			return -ENODEV;
		}

		err = gpio_pin_configure(gpioDevs[i], gpioPins[i].number,
					gpioPins[i].mode | flags);

		if (err) {
			//LOG_ERR("Cannot configure button gpio");
			return err;
		}
	}

    devSpiAccMem = (struct device*)DEVICE_DT_GET(DT_NODELABEL(spi1));
	if (!device_is_ready(devSpiAccMem)) {
		printk("SPI device %s is not ready\n", devSpiAccMem->name);
		return -1;
	}

    mb85rs1WriteSr(srVal);
    
    mb85rs1ReadId(&pBuf);

    return 0;
}

uint8_t mb85rs1ReadId(uint32_t* pBuf){

    uint8_t cmd[1] = {MB85RS1_CMD_RDID};
    mb85rs1TransceiveCmd(1, cmd, 0, NULL, 4, (uint8_t*)pBuf);

    return 0;
}

uint8_t mb85rs1ReadSr(uint8_t* pBuf){
 
    uint8_t cmd[1] = {MB85RS1_CMD_RDSR};
    mb85rs1TransceiveCmd(1, cmd, 0, NULL, 1, pBuf);

    return 0;
}

uint8_t mb85rs1WriteSr(uint8_t* pBuf){
    
    uint8_t cmd[1] = {MB85RS1_CMD_WRSR};
    mb85rs1TransceiveCmd(1, cmd, 1, pBuf, 0, NULL);

    return 0;
}

uint8_t mb85rs1WriteEnable(void){

    uint8_t cmd[1] = {MB85RS1_CMD_WREN};

    mb85rs1TransceiveCmd(1, cmd, 0, NULL, 0, NULL);
    
    return 0;
}

uint8_t mb85rs1WriteDisable(void){

    uint8_t cmd[1] = {MB85RS1_CMD_WRDI};

    mb85rs1TransceiveCmd(1, cmd, 0, NULL, 0, NULL);
    
    return 0;
}

uint8_t mb85rs1ReadData(uint32_t addr, uint8_t* pBuf, uint32_t len){

    uint8_t cmd[4] = {MB85RS1_CMD_READ, 
                        (addr >> 16) & 0xff, 
                        (addr >> 8) & 0xff, 
                        addr & 0xff};

    mb85rs1TransceiveCmd(4, cmd, 0, NULL, len, pBuf);

    return 0;

}

uint8_t mb85rs1WriteData(uint32_t addr, uint8_t* pBuf, uint32_t len){

     uint8_t cmd[4] = {MB85RS1_CMD_WRITE, 
                        (addr >> 16) & 0xff, 
                        (addr >> 8) & 0xff, 
                        addr & 0xff};

    mb85rs1WriteEnable();

    mb85rs1TransceiveCmd(4, cmd, len, pBuf, 0, NULL);

    return 0;

}

uint8_t mb85rs1ReadDataFast(uint32_t addr, uint8_t* pBuf, uint32_t len){

     uint8_t cmd[5] = {MB85RS1_CMD_FSTRD,
                        (addr >> 16) & 0xff, 
                        (addr >> 8) & 0xff, 
                        addr & 0xff, 0};

    mb85rs1TransceiveCmd(5, cmd, 0, NULL, len, pBuf);

    return 0;
}

uint8_t mb85rs1SetSleep(bool mode){
    
    uint8_t cmd[1] = {MB85RS1_CMD_WRDI};

    if(mode){
        mb85rs1TransceiveCmd(1, cmd, 0, NULL, 0, NULL);
    }else{

        SRAM_CS_ENABLE;
        k_sleep(K_MSEC(1));
        SRAM_CS_DISABLE;
    }
    
    return 0;
}