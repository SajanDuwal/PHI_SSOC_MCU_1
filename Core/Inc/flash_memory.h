/*
 * flash_memory.h
 *
 *  Created on: Dec 16, 2024
 *      Author: sajanduwal
 */

#ifndef INC_FLASH_MEMORY_H_
#define INC_FLASH_MEMORY_H_

#include "main.h"
extern SPI_HandleTypeDef hspi2;

/*external variables or structures*/

//Basic Memory Operations
#define		WRITE_ENABLE					0x06
#define		WRITE_DISABLE					0x04

//Read Manufacturer ID
#define 	READ_ID							0x9E

//Read Operations
#define 	READ							0x03
#define		BYTE_4_READ						0x13	//4-byte address

//Write or Program operations
#define		PAGE_PROGRAM					0x02
#define		BYTE_4_PAGE_PROGRAM				0x12	//4-byte address

//Erase operations
#define		SECTOR_ERASE					0xD8
#define		SUBSECTOR_ERASE_32KB			0x52
#define		SUBSECTOR_ERASE_4KB				0x20

#define 	PAGE_SIZE						256

/* Typedef structures */

/*to check the device ID of Flash memory*/
typedef struct {
	uint8_t MAN_ID;
	uint8_t M_TYPE;
	uint8_t M_CAP;
	uint8_t REM_BYTES;
	uint8_t EXT_ID;
	uint8_t DEV_INFO;
	uint8_t UID[14];
} DEVICE_ID;

void delay_us(uint16_t ms);

//################# FLASH MEMORY OPERATION ############################

/* Read write Operations */
void Read_ID(SPI_HandleTypeDef *SPI, DEVICE_ID *FM_ID);
void Bulk_Read(SPI_HandleTypeDef *SPI, uint32_t address, uint8_t *data,
		uint16_t size);
void Page_Write(SPI_HandleTypeDef *SPI, uint32_t address, uint8_t *data,
		uint16_t size);

void Page_Write_4B(SPI_HandleTypeDef *SPI, uint32_t address, uint8_t *data,
		uint16_t size);

void Bulk_Read_4B(SPI_HandleTypeDef *SPI, uint32_t address, uint8_t *data,
		uint16_t size);

/* Erase operations */
uint8_t Sector_Erase(SPI_HandleTypeDef *SPI, uint32_t address,
		uint8_t sector_size);

/* CS setting */
void FM_Disable(SPI_HandleTypeDef *SPI);
void FM_Enable(SPI_HandleTypeDef *SPI);

//To reset memory through software
void Write_Enable(SPI_HandleTypeDef *SPI);

#endif /* INC_FLASH_MEMORY_H_ */
