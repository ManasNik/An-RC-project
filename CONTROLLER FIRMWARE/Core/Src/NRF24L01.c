#include "stm32f1xx_hal.h"
#include "NRF24L01.h"

#define NRF24_CE_PORT GPIOB
#define NRF24_CE_PIN GPIO_PIN_1

#define NRF24_CS_PORT GPIOB
#define NRF24_CS_PIN GPIO_PIN_0

extern SPI_HandleTypeDef hspi1;

void CS_select(){
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_RESET);
}

void CS_deselect(){
	HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_SET);
}

void CE_enable(){
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

void CE_disable(){
	HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}


// Write a single byte to a register
void nrf24_writeReg(uint8_t reg, uint8_t data){
	uint8_t buf[2];

	buf[0] = reg | 1<<5;
	buf[1] = data;

	CS_select();

	HAL_SPI_Transmit(&hspi1, buf, 2, 1000);

	CS_deselect();
}

// Write a single byte starting from a particular register
void nrf24_writeRegMulti(uint8_t reg, uint8_t* data,int size){
	uint8_t buf[2];

	buf[0] = reg | 1<<5;

	CS_select();

	HAL_SPI_Transmit(&hspi1, buf, 1, 100);
	HAL_SPI_Transmit(&hspi1, data, size, 1000);

	CS_deselect();
}

// Read a single byte from a register
uint8_t nrf24_readReg(uint8_t reg){
	uint8_t data = 0;

	CS_select();

	HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
	HAL_SPI_Receive(&hspi1, &data, 1, 100);

	CS_deselect();

	return data;
}

// Read multiple bytes from the register
void nrf24_readReg_Multi(uint8_t reg,uint8_t* data,int size){
	CS_select();

	HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
	HAL_SPI_Receive(&hspi1, data, size, 1000);

	CS_deselect();
}

// Send the commands to NRF
void nrf24_sendCmd(uint8_t cmd){
	CS_select();

	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);

	CS_deselect();
}

void nrf24_init(){
	// Disable the chip before configuring it
	CE_disable();

	nrf24_writeReg(CONFIG, 0); // will be configured later

	nrf24_writeReg(EN_AA, 0); // No auto ACK

	nrf24_writeReg(EN_RXADDR, 0); // Not enabling any data pipe right now

	nrf24_writeReg(SETUP_AW, 0x03); // 5 bytes for TX/RX address

	nrf24_writeReg(SETUP_RETR, 0); // No retransmission

	nrf24_writeReg(RF_CH, 0); // will be setup during RX or TX

	nrf24_writeReg(RF_SETUP, 0x0E); // power = 0 db, data rate = 2mbps

	// Enable the chip after configuring it
	CE_enable();
}

// Setup TX mode

void nrf24_TXmode(uint8_t *address, uint8_t channel){
	// Disable the chip before configuring it
	CE_disable();

	nrf24_writeReg(RF_CH, channel); // Select the channel

	nrf24_writeRegMulti(TX_ADDR, address, 5); // Write the TX address

	// Power up the device
	uint8_t config = nrf24_readReg(CONFIG);
	config |= (1<<1);
	config &= ~(1 << 0);   // write 0 in the PRIM_RX, and 1 in the PWR_UP, and all other bits are masked
	nrf24_writeReg(CONFIG, config);

	// Enable the chip after configuring it
	CE_enable();
}

uint8_t nrf24_Transmit(uint8_t* data,uint8_t size){
	uint8_t cmdToSend = 0;

	// Select the device
	CS_select();

	// payload command
	cmdToSend = W_TX_PAYLOAD;
	HAL_SPI_Transmit(&hspi1, &cmdToSend, 1, 100);

	// Send the payload
	HAL_SPI_Transmit(&hspi1, data, size, 1000);

	// deselect the device
	CS_deselect();

	HAL_Delay(1);

	uint8_t fifoStatus = nrf24_readReg(FIFO_STATUS);

	if((fifoStatus & (1<<4)) && (!(fifoStatus & (1<<5)))){
		cmdToSend = FLUSH_TX;

		nrf24_sendCmd(cmdToSend);

		return 1;
	}

	return 0;
}

void nrf24_RXmode(uint8_t* address,uint8_t payload_size, uint8_t channel){
	// Disable the chip before configuring it
	CE_disable();

	nrf24_writeReg(RF_CH, channel); // Select the channel

	// Select data pipe 1
	uint8_t en_rxaddr = nrf24_readReg(EN_RXADDR);
	en_rxaddr |= (1<<1);
	nrf24_writeReg(EN_RXADDR,en_rxaddr);

	nrf24_writeRegMulti(RX_ADDR_P1, address, 5); // Write the TX address

	nrf24_writeReg(RX_PW_P1, payload_size); // Payload size is adjustable

	// Power up the device in RX mode
	uint8_t config = nrf24_readReg(CONFIG);
	config = config | (1<<1) | (1<<0);
	nrf24_writeReg(CONFIG, config);

	// Enable the chip after configuring it
	CE_enable();
}

uint8_t isDataAvailable(int pipenum){
	uint8_t status = nrf24_readReg(STATUS);

	// Note: The condition changes if multiple data pipes are used!!
	if((status & (1<<6))){
		nrf24_writeReg(STATUS, (1<<6));

		return 1;
	}

	return 0;
}


uint8_t nrf24_Receive(uint8_t* data, uint8_t size){
	uint8_t cmdToSend = 0;

		// Select the device
		CS_select();

		// payload command
		cmdToSend = R_RX_PAYLOAD;
		HAL_SPI_Transmit(&hspi1, &cmdToSend, 1, 100);

		// Send the payload
		HAL_SPI_Receive(&hspi1, data, size, 1000);

		// deselect the device
		CS_deselect();

		HAL_Delay(1);

		//cmdToSend = FLUSH_RX;

		//nrf24_sendCmd(cmdToSend);
}
