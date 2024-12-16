/*
 * flash_memory.c
 *
 *  Created on: Dec 16, 2024
 *      Author: sajanduwal
 */

#include "main.h"
#include "flash_memory.h"

void Read_ID(SPI_HandleTypeDef *SPI, DEVICE_ID *FM_ID) {
	uint8_t cmd = READ_ID;
	DEVICE_ID buff;
	uint8_t data[20];
	int i;
	FM_Enable(SPI);
	delay_us(1);
	HAL_SPI_Transmit(SPI, &cmd, 1, 300);
	HAL_SPI_Receive(SPI, data, 20, 1000);
	delay_us(1);
	FM_Disable(SPI);
	delay_us(500);
	buff.MAN_ID = data[0];
	buff.M_TYPE = data[1];
	buff.M_CAP = data[2];
	buff.REM_BYTES = data[3];
	buff.EXT_ID = data[4];
	buff.DEV_INFO = data[5];
	for (i = 6; i < 20; i++) {
		buff.UID[i] = data[i];
	}
	*FM_ID = buff;
	return;
}

/*
 * @brief	to write to a page(256 bytes) of the FM 3-byte-addressing mode
 *
 * @param	address		4 byte starting address from which data is to be read
 * 			*data		data to be written into the address
 * 			size		size of the data
 *@retval	none
 */
void Page_Write(SPI_HandleTypeDef *SPI, uint32_t address, uint8_t *data,
		uint16_t size) {
	uint8_t cmd = PAGE_PROGRAM;
	uint8_t command[3];
	while (size > PAGE_SIZE) {
		command[0] = (uint8_t) (address >> 16 & 0xff);
		command[1] = (uint8_t) (address >> 8 & 0xff);
		command[2] = (uint8_t) (address & 0xff);
		Write_Enable(SPI);
		FM_Enable(SPI);
		delay_us(1);
		HAL_SPI_Transmit(SPI, &cmd, 1, 100);
		HAL_SPI_Transmit(SPI, command, 3, 200);
		HAL_SPI_Transmit(SPI, data, PAGE_SIZE, 200);
		FM_Disable(SPI);
		delay_us(5);
		size = size - PAGE_SIZE;
		data = data + 256;
		address = address + 256;
	}
	command[0] = (uint8_t) (address >> 16 & 0xff);
	command[1] = (uint8_t) (address >> 8 & 0xff);
	command[2] = (uint8_t) (address & 0xff);
	Write_Enable(SPI);
	FM_Enable(SPI);
	delay_us(1);
	HAL_SPI_Transmit(SPI, &cmd, 1, 100);
	HAL_SPI_Transmit(SPI, command, 3, 200);
	HAL_SPI_Transmit(SPI, data, size, 200);
	FM_Disable(SPI);
	delay_us(5);
	return;
}

/*
 * @brief	Functions to Erase the Sectors of Corresponding size
 * 			Suffix gives the size of the Sector e.g. 32 = 32 KB subsector
 *
 * 	@param	address		any address(starting, middle or end) of the sector or subsector
 *
 * 	@retval 1/2/3/4 if the operation is successful
 * 			0 if the operation failed or input was invalid
 */
uint8_t Sector_Erase(SPI_HandleTypeDef *SPI, uint32_t address,
		uint8_t sector_size) {
	uint8_t addr[4];
	uint8_t cmd = 0;
	addr[0] = (uint8_t) (address >> 24 & 0xff);
	addr[1] = (uint8_t) (address >> 16 & 0xff);
	addr[2] = (uint8_t) (address >> 8 & 0xff);
	addr[3] = (uint8_t) (address & 0xFF);
	switch (sector_size) {
	case 64:
		cmd = SECTOR_ERASE;
		Write_Enable(SPI);
		FM_Enable(SPI);
		delay_us(1);
		HAL_SPI_Transmit(SPI, &cmd, 1, 100);
		HAL_SPI_Transmit(SPI, addr, 3, 200);
		FM_Disable(SPI);
		delay_us(1050);
		return 1;
	case 32:
		cmd = SUBSECTOR_ERASE_32KB;
		Write_Enable(SPI);
		FM_Enable(SPI);
		HAL_SPI_Transmit(SPI, &cmd, 1, 100);
		HAL_SPI_Transmit(SPI, addr, 3, 200);
		FM_Disable(SPI);
		delay_us(1000);
		return 2;
	case 4:
		cmd = SUBSECTOR_ERASE_4KB;
		Write_Enable(SPI);
		FM_Enable(SPI);
		HAL_SPI_Transmit(SPI, &cmd, 1, 100);
		HAL_SPI_Transmit(SPI, addr, 3, 200);
		FM_Disable(SPI);
		delay_us(500);
		return 3;
	default:
		return 0;
	}
}

void Write_Enable(SPI_HandleTypeDef *SPI) {
	uint8_t cmd = WRITE_ENABLE;
	FM_Enable(SPI);
	delay_us(1);
	HAL_SPI_Transmit(SPI, &cmd, 1, 500);
	FM_Disable(SPI);
	delay_us(10);
}

/*
 * @brief	function to read array of data of any size
 *
 * @param	SPI			pointer to the handle of SPI connected to Flash from which to read data
 * @param	address		address from which the data is to be read
 * 			*data		pointer to store the data which is read from the corresponding address
 * 			size		size of the data to be read
 */
void Bulk_Read(SPI_HandleTypeDef *SPI, uint32_t address, uint8_t *data,
		uint16_t size) {
	uint8_t cmd[4];
	cmd[0] = READ;
	cmd[1] = (uint8_t) (address >> 16 & 0xff);
	cmd[2] = (uint8_t) (address >> 8 & 0xff);
	cmd[3] = (uint8_t) (address & 0xff);
	FM_Enable(SPI);
	HAL_SPI_Transmit(SPI, cmd, 4, 100);
	HAL_SPI_Receive(SPI, data, size, 200);
	FM_Disable(SPI);
	delay_us(5);
	return;
}

void FM_Enable(SPI_HandleTypeDef *SPI) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, RESET);
	delay_us(1);
}

void FM_Disable(SPI_HandleTypeDef *SPI) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);
	delay_us(1);
}

/*
 @brief    to write to a page(256 bytes) of the FM 4-byte-addressing-mode*
 @param    address        4 byte starting address from which data is to be read
 *data        data to be written into the address
 size        size of the data
 *@retval    none
 */

void Page_Write_4B(SPI_HandleTypeDef *SPI, uint32_t address, uint8_t *data,
		uint16_t size) {
	uint8_t cmd = BYTE_4_PAGE_PROGRAM;
	uint8_t command[4];
	while (size > PAGE_SIZE) {
		command[0] = (uint8_t) (address >> 24 & 0xff);
		command[1] = (uint8_t) (address >> 16 & 0xff);
		command[2] = (uint8_t) (address >> 8 & 0xff);
		command[3] = (uint8_t) (address & 0xFF);
		Write_Enable(SPI);
		FM_Enable(SPI);
		delay_us(1);
		HAL_SPI_Transmit(SPI, &cmd, 1, 100);
		HAL_SPI_Transmit(SPI, command, 4, 200);
		HAL_SPI_Transmit(SPI, data, PAGE_SIZE, 200);
		FM_Disable(SPI);
		delay_us(5);
		size = size - PAGE_SIZE;
		data = data + 256;
		address = address + 256;
	}
	command[0] = (uint8_t) (address >> 24 & 0xff);
	command[1] = (uint8_t) (address >> 16 & 0xff);
	command[2] = (uint8_t) (address >> 8 & 0xff);
	command[3] = (uint8_t) (address & 0xFF);
	Write_Enable(SPI);
	FM_Enable(SPI);
	delay_us(1);
	HAL_SPI_Transmit(SPI, &cmd, 1, 50);
	HAL_SPI_Transmit(SPI, command, 4, 100);
	delay_us(1);
	HAL_SPI_Transmit(SPI, data, size, 200);
	FM_Disable(SPI);
	delay_us(5);
	return;
}

/*
 * @brief	similar function as Bulk_Read but 4 byte address mode for the FM
 *
 *
 * @param	SPI			pointer to the handle of SPI connected to Flash from which to read data
 * * @param	address		address from which the data is to be read
 * 			*data		pointer to store the data which is read from the corresponding address
 * 			size		size of the data to be read
 */
void Bulk_Read_4B(SPI_HandleTypeDef *SPI, uint32_t address, uint8_t *data,
		uint16_t size) {
	uint8_t cmd[5];
	cmd[0] = BYTE_4_READ;
	cmd[1] = (uint8_t) (address >> 24 & 0xff);
	cmd[2] = (uint8_t) (address >> 16 & 0xff);
	cmd[3] = (uint8_t) (address >> 8);
	cmd[4] = (uint8_t) (address);
	FM_Enable(SPI);
	HAL_SPI_Transmit(SPI, cmd, 5, 100);
	HAL_SPI_Receive(SPI, data, size, 200);
	FM_Disable(SPI);
	delay_us(5);
	return;
}
