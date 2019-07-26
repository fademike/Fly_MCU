
/*
 *  Created on: 17 04 2019 ã.
 *      Author: fademike
 *      GIT: https://github.com/fademike
*/

#include "stm32f1xx_hal.h"

#include "main.h"
#include "uSD.h"

#include <string.h>		// for memcpy()

extern SPI_HandleTypeDef hspi2;

int32_t CADR_TYPE = SD_ERROR;


unsigned char DataCSD[17] = {0,};


#define GEBUG_uSD 0

uint8_t CRC7(uint8_t * chr, int cnt) {
	int i,a;
	unsigned char crc,Data;

	crc=0;
	for (a=0;a<cnt;a++) {
		Data=chr[a];
		for (i=0;i<8;i++) {
			crc <<= 1;

			if ((Data & 0x80)^(crc & 0x80))crc ^=0x09;
			Data <<= 1;
		}
	}
	crc=(crc<<1)|1;
	return(crc);
}


uint32_t uSD_GetCSDStructure(void)
{
	CSD_V1_0 * csd = (CSD_V1_0 *) &DataCSD[0];
	return csd->CSD_STRUCTURE;
}

uint32_t uSD_GetSize(void)
{
	if ((CADR_TYPE == SD_VER2HC) || (CADR_TYPE == SD_VER2SC)) {
		CSD_V2_0 * csd = (CSD_V2_0 *) &DataCSD[0];

		 //Printf("c_struct: %d\n\r", csd->CSD_STRUCTURE);
		 //Printf("READ_BL_LEN: %d\n\r", csd->READ_BL_LEN);
		 //Printf("READ_BL_PARTIAL: %d\n\r", csd->READ_BL_PARTIAL);
		 //Printf("WRITE_BLK_MISALIGN: %d\n\r", csd->WRITE_BLK_MISALIGN);
		 //Printf("READ_BLK_MISALIGN: %d\n\r", csd->READ_BLK_MISALIGN);
		 //Printf("C_SIZE: %d\n\r", (csd->C_SIZE_2<<(6+8))|(csd->C_SIZE_1<<6)|(csd->C_SIZE_0>>2));
		 //Printf("ERASE_BLK_EN: %d\n\r", csd->ERASE_BLK_EN);
		 //Printf("SECTOR_SIZE: %d\n\r", (csd->SECTOR_SIZE_1<<1)|(csd->SECTOR_SIZE_0&0x1));
		 //Printf("WRITE_BL_LEN: %d\n\r", (csd->WRITE_BL_LEN_1<<2)|(csd->WRITE_BL_LEN_0&0x3));
		 //Printf("WRITE_BL_PARTIAL): %d\n\r", csd->WRITE_BL_PARTIAL);
		 //Printf("FILE_FORMAT_: %d\n\r", csd->FILE_FORMAT);

		int size = (csd->C_SIZE_2<<(6+8))|(csd->C_SIZE_1<<6)|(csd->C_SIZE_0);
		return size;
	}
	else if (CADR_TYPE == SD_VER1) {
		CSD_V1_0 * csd = (CSD_V1_0 *) &DataCSD[0];
		int size = (csd->C_SIZE_2<<(2+8))|(csd->C_SIZE_1<<2)|(csd->C_SIZE_0);
		return size;
	}

	return 0;
}

enum  {
	RESP_NO = 0,
	RESP_R1 = 1,
	RESP_R1b = 2,
	RESP_R2 = 3,
	RESP_R3 = 4,
	RESP_R6 = 5,
	RESP_R5 = 6,
	RESP_CSD = 7,
	RESP_R7 = 8
}RESP_TYPE;


//1 - enable; 0 - disable
#define CS(x)    if (1){ 	if (x == 0) HAL_GPIO_WritePin(SDCARD_SS_GPIO_Port, SDCARD_SS_Pin, GPIO_PIN_SET); \
							else  HAL_GPIO_WritePin(SDCARD_SS_GPIO_Port, SDCARD_SS_Pin, GPIO_PIN_RESET);	}



uint8_t SPI_TXRX(uint8_t Data)
{
	unsigned char dataout = 0;
	HAL_SPI_TransmitReceive(&hspi2, &Data, &dataout, 1, 100);
	return dataout;
}


int GET_Data(uint8_t * buf, uint32_t len)
{
	int i=0;
	for (i=0;i<len;i++) buf[i] = SPI_TXRX(0xFF);
	return len;
}

int GET_WaitNoToken(uint8_t token)
{
	int timeout = 10000;
	uint8_t res = 0;
	while((res = SPI_TXRX(0xFF)) == token)
	{
		//HAL_Delay(10);
	   if(timeout-- <=0) {if (GEBUG_uSD)Printf("Timout ");return res;}
	}
	return res;
}
int GET_WaitToken(uint8_t token)
{
	int timeout = 10000;
	uint8_t res = 0;
	while((res = SPI_TXRX(0xFF)) != token)
	{
		//HAL_Delay(10);
	   if(timeout-- <=0) {if (GEBUG_uSD)Printf("Timout ");return res;}
	}
	return res;
}

int GET_RESP(uint8_t * resp, uint8_t len)
{
	uint8_t res = 0;
	res = GET_WaitNoToken(0xFF);

	GET_Data(resp, len);

	SPI_TXRX(0xFF);

	return res;
}


int SEND_DATA(uint8_t * buf, uint32_t len)
{
	int i=0;
	for (i=0;i<len;i++) SPI_TXRX(buf[i]);
	return len;
}

int SEND_cmd(uint8_t cmd, uint32_t arg)
{
	if (GEBUG_uSD)Printf("cmd %d, ", cmd);
	unsigned char data[6] = {cmd|0x40, arg >> 24, arg >> 16, arg >> 8, arg, 0};

	data[5] = CRC7(data, 5);
	SEND_DATA(data, 6);

	return cmd;
}

int uSD_SendCMD(uint8_t cmd, uint32_t arg, uint8_t resp_type, uint8_t * resp)//SEND_CMD
{
	if (GEBUG_uSD)Printf("cmd %d, ", cmd);
	unsigned char data[6] = {cmd|0x40, arg >> 24, arg >> 16, arg >> 8, arg, 0};
	int ret = 0;
	data[5] = CRC7(data, 5);
	CS(1);

	SEND_DATA(data, 6);


	if (resp_type == RESP_R1) ret = GET_RESP(resp, 1);			//for SPI
	else if (resp_type == RESP_R1b) ret = GET_RESP(resp, 1);
	else if (resp_type == RESP_R2) ret = GET_RESP(resp, 2);
	else if (resp_type == RESP_R3) ret = GET_RESP(resp, 5);
	//else if (resp_type == RESP_R6) ret = GET_RESP(resp, 6);
	else if (resp_type == RESP_R7) ret = GET_RESP(resp, 5);
	else if (resp_type == RESP_CSD) ret = GET_RESP(resp, 16+1);
	else ret = GET_RESP(resp, 0);

	CS(0);

	if (GEBUG_uSD)Printf("\n\r");

	return ret;
}

int uSD_Read(int sector, uint8_t * buff)
{
	if (GEBUG_uSD)Printf("uSD_Read %d\n\r",sector);

	CS(1);
	SEND_cmd(17, sector);

	if (GET_WaitToken(0xFE) != 0xFE){CS(0);if (GEBUG_uSD)Printf("ERRD2\n\r");return -2;}

	GET_Data(buff, 512);

	SPI_TXRX(0xFF);
	SPI_TXRX(0xFF);
	SPI_TXRX(0xFF);
	CS(0);

	return 0;
}




int uSD_Write(int sector, uint8_t * buff)
{
	 //int i=0;
	 uint8_t resp[17];
	 //int timeout = 0;

	 if (GEBUG_uSD)Printf("uSD_Write %d\n\r",sector);

	 if (uSD_SendCMD(24, sector, RESP_R1, resp) == 0xFF) {if (GEBUG_uSD)Printf("ERR25\n\r"); return -1;}

	  CS(1);
	  SPI_TXRX(0xFE);

	  SEND_DATA(buff, 512);

	  SPI_TXRX(0xFF);
	  SPI_TXRX(0xFF);

	  char temp = SPI_TXRX(0xFF);

	  if( (temp & 0x1F) != 0x05) {CS(0);if (GEBUG_uSD)Printf("ERR26\n\r");return -1;}

	  if (GET_WaitNoToken(0x00) == 0x00) {CS(0);if (GEBUG_uSD)Printf("ERR27\n\r");return -1;}

	  //CS(0);
	  //SPI_TXRX(0xFF);
	  //CS(1);

	  //if (GET_WaitNoToken(0x00) == 0x00) {CS(0);if (GEBUG_uSD)Printf("ERR28\n\r");return -1;}
	  CS(0);

	  if (GEBUG_uSD)Printf("Write FIN\n\r");
	  return 0;
}




int uSD_Init(void)
{
	int i=0;

	uint8_t resp[17];

	if (GEBUG_uSD)Printf("uSD_Init\n\r");

	 CS(0);
	 HAL_Delay(50);
	 for (i = 0; i<10; i++) SPI_TXRX(0xFF);
	 HAL_Delay(50);

	if (uSD_SendCMD(0, 0, RESP_NO, resp) == 0xFF) {if (GEBUG_uSD)Printf("ERR1\n\r"); return -1;} //IDLE

	 if ((uSD_SendCMD(8, 0x01AA, RESP_R7, resp)) == 0xFF)
	 {  //VER1.0
		 if (GEBUG_uSD)Printf("VER1.0\n\r");
		 if (uSD_SendCMD(55, 0, RESP_R1, resp) == 0xFF) if (GEBUG_uSD)Printf("ERRX1\n\r");
		 if (uSD_SendCMD(41, 0, RESP_R3, resp) == 0xFF)
		 {

		   if (uSD_SendCMD(1, 0, RESP_R1, resp) == 0xFF) {if (GEBUG_uSD)Printf("ERR2\n\r"); return -1;}
		   CADR_TYPE = SD_MMC;
		 } else{}
		 CADR_TYPE = SD_VER1;
	 }
	 else
	 {  //VER2.0
		 if (GEBUG_uSD)Printf("VER2.0\n\r");
		 int ret = 1;
		 int timeout = 10000;
		 do{
			 if ((ret = uSD_SendCMD(55, 0, RESP_NO, resp)) == 0xFF) {if (GEBUG_uSD)Printf("ERRX2\n\r"); return -1;}
			 if (uSD_SendCMD(41, 0x40000000, RESP_R1, resp) == 0xFF) {if (GEBUG_uSD)Printf("ERR3\n\r"); return -1;}
		 }while((ret != 0) && (timeout-->0));
		 if (timeout == 0){if (GEBUG_uSD)Printf("timout err\n\r"); return -1;}
		 if ((ret = uSD_SendCMD(58, 0, RESP_R3, resp)) == 0xFF) {if (GEBUG_uSD)Printf("ERR4\n\r"); return -1;}

		 if(resp[0] & 0x40) CADR_TYPE = SD_VER2HC;
		 else CADR_TYPE = SD_VER2SC;

	 }

	 if (CADR_TYPE != SD_VER2HC)
		 if (uSD_SendCMD(16, 0x200, RESP_R1, resp) == 0xFF) {if (GEBUG_uSD)Printf("ERR5\n\r"); return -1;}


	 CS(1);
	 SEND_cmd(9, 0);
	 if (GET_WaitNoToken(0xFF) == 0xFF){if (GEBUG_uSD)Printf("GET_WaitNoToken\n\r");  return -1;}
	 if (GET_WaitToken(0xFE) !=0xFE){if (GEBUG_uSD)Printf("GET_WaitToken\n\r");  return -1;}
	 GET_Data(resp, 16);
	 CS(0);

	 memcpy(DataCSD, resp, 16);

	 int size = uSD_GetSize();	// size*512*4 = databyte
	 Printf("size is %d\n\r", size);


	 if  (uSD_GetCSDStructure() == 0) {if(CADR_TYPE != SD_VER1)CADR_TYPE = SD_ERR;}
	 else if  (uSD_GetCSDStructure() == 1) {if ((CADR_TYPE != SD_VER2HC) && (CADR_TYPE != SD_VER2SC))CADR_TYPE = SD_ERR;}
	 if (size<=0) CADR_TYPE = SD_ERR;

	 if (GEBUG_uSD)Printf("FIN, card=%d\n\r", CADR_TYPE);
	 //return -1;
	 return CADR_TYPE;
}






void uSD_Test(void)
{
	if(1)  while(1){

		int i=0;


		FATFS fs;
		FRESULT res;
		DIR dirs;
		FILINFO finfo;


		//disk_initialize();
		res=f_mount(0, &fs);
		Printf("Mounted is finished, %d\r\n", res);
		//	   	res=f_opendir(&dirs, "0:/");//?????
		//	 Printf("Opendir is finished, %d\r\n", res);
		//   	res=f_readdir(&dirs, &finfo);
		// Printf("Readdir is finished, %d\r\n", res);


		res=f_opendir(&dirs, "0:/");//?????
		Printf("Opendir is finished, %d\r\n", res);

		//int i=0;
		for (i=0;i<10;i++){
		res=f_readdir(&dirs, &finfo);
		Printf("Readdir fin:%d, name:%s, size:%d, date:%d, time:%d, fatrib:%d\r\n", res, finfo.fname, finfo.fsize, finfo.fdate, finfo.ftime, finfo.fattrib);
		}


		FIL file;

		res = f_open(&file, "0:123.txt", FA_READ | FA_OPEN_EXISTING);

		Printf("f_open read is finished, %d\r\n", res);

		if (res == 0){
		unsigned int len = 0;
		unsigned char bbb[512];
		res = f_read(&file, bbb, 512, &len);
		Printf("read is finished, %d,  %d\r\n", res, len);
		bbb[len] = '\0';
		Printf("get str=%s\n\r", bbb);
		f_close(&file);

		//if (res == 0) f_close(&file);

		}



		res = f_open(&file, "0:555.txt", FA_WRITE | FA_CREATE_ALWAYS);

		Printf("f_open is finished, %d\r\n", res);

		if (res == 0){
		unsigned int len = 0;
		char str[] = "write 555.txt\r\nHello from stm32";
		res = f_write(&file, str, strlen(str), &len);

		Printf("writed is finished, %d,  %d\r\n", res, len);

		f_close(&file);

		//if (res == 0) f_close(&file);

		}






		//}

		while(1){};
		//uSD_TEST();

		HAL_Delay(5000);
	  };

}
