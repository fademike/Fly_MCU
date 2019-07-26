/*
 * MPU9250.c
 *
 *  Created on: 08 θώνÿ 2018 γ.
 *      Author: fademike
 *      GIT: https://github.com/fademike
 */

#include "MPU9250.h"
#include "main.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define DEBUG 0

#define DEVID (0x68<<1)
#define MAGID (0x0C<<1)
#define BMPID (0x76<<1)

pos_struct Est_A, Est_G, Est_M;
//pos_struct Est_G;
//pos_struct Est_M;
signed short MPU_temp = 0;
signed int BMP_temp = 0;
signed int BMP_press = 0;

float magCorrect_ASA[3] = {1.0, 1.0, 1.0};
const float MagHardCorrect[3] = {0.0, -150.0, -200.0};
//float MPU_MAG_asay;
//float MPU_MAG_asaz;

uint8_t aTxBuffer[1];

#define WHO_AM_I 0x75

int MPU_Init(void)
{
	  if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), WHO_AM_I, 1, aTxBuffer, 1, 1000) != HAL_OK)return -1;

	  if ((aTxBuffer[0] != 0x71) && (aTxBuffer[0] != 0x73)) return -2;

	    //Set magnit
	  aTxBuffer[0] = 0x00;
	  if (HAL_I2C_Mem_Write(&hi2c1, (DEVID+0), 0x6B, 1, aTxBuffer, 1, 1000) != HAL_OK)return -1;
	  if (HAL_I2C_Mem_Write(&hi2c1, (DEVID+0), 0x6A, 1, aTxBuffer, 1, 1000) != HAL_OK)return -1;
	  aTxBuffer[0] = 0x02;
	  if (HAL_I2C_Mem_Write(&hi2c1, (DEVID+0), 0x37, 1, aTxBuffer, 1, 1000) != HAL_OK)return -1;



	  if (HAL_I2C_Mem_Read(&hi2c1, (MAGID+1), 0x01, 1, aTxBuffer, 1, 1000) != HAL_OK)return -1;
      if (aTxBuffer[0] != 154) return -3;

  	unsigned char varX;
  	unsigned char varY;
  	unsigned char varZ;
    if (HAL_I2C_Mem_Read(&hi2c1, (MAGID+1), AK8963_ASAX, 1, &varX, 1, 1000) != HAL_OK)return -1;
    if (HAL_I2C_Mem_Read(&hi2c1, (MAGID+1), AK8963_ASAY, 1, &varY, 1, 1000) != HAL_OK)return -1;
    if (HAL_I2C_Mem_Read(&hi2c1, (MAGID+1), AK8963_ASAZ, 1, &varZ, 1, 1000) != HAL_OK)return -1;

  	magCorrect_ASA[0] = ((((varX-128)*0.5)/128)+1);
  	magCorrect_ASA[1] = ((((varY-128)*0.5)/128)+1);
  	magCorrect_ASA[2] = ((((varZ-128)*0.5)/128)+1);

    if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xD0, 1, aTxBuffer, 1, 1000) != HAL_OK)return -1;
    //if (aTxBuffer[0] != 0xD0) return -4;


	  return 0;
}




/*
int fd; // Device file handle of mpu9250 DEVID (0x68<<1)
int md; // Device file handle of ak8963	 MAGID (0x0C<<1)
int bd; // Device file handle of bmp280  BMPID (0x76<<1)
*/







int Get_acc(void)
{
	unsigned char d[6];

    if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), ACCEL_XOUT_H, 1, &d[1], 1, 1000) != HAL_OK)return -1;
    if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), ACCEL_XOUT_L, 1, &d[0], 1, 1000) != HAL_OK)return -1;
    if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), ACCEL_YOUT_H, 1, &d[3], 1, 1000) != HAL_OK)return -1;
    if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), ACCEL_YOUT_L, 1, &d[2], 1, 1000) != HAL_OK)return -1;
    if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), ACCEL_ZOUT_H, 1, &d[5], 1, 1000) != HAL_OK)return -1;
    if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), ACCEL_ZOUT_L, 1, &d[4], 1, 1000) != HAL_OK)return -1;

	short x = *(short *)&d[0];
	short y = *(short *)&d[2];
	short z = *(short *)&d[4];

	if(DEBUG) printf("ACC x=%d, y=%d, z=%d\n\r", x, y, z);

	Est_A.x = x;
	Est_A.y = y;
	Est_A.z = z;

	return 0;
}


int Get_gyro(void)
{
	unsigned char d[6];

    if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), GYRO_XOUT_H, 1, &d[1], 1, 1000) != HAL_OK)return -1;
    if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), GYRO_XOUT_L, 1, &d[0], 1, 1000) != HAL_OK)return -1;
    if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), GYRO_YOUT_H, 1, &d[3], 1, 1000) != HAL_OK)return -1;
    if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), GYRO_YOUT_L, 1, &d[2], 1, 1000) != HAL_OK)return -1;
    if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), GYRO_ZOUT_H, 1, &d[5], 1, 1000) != HAL_OK)return -1;
    if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), GYRO_ZOUT_L, 1, &d[4], 1, 1000) != HAL_OK)return -1;

	short x = *(short *)&d[0];
	short y = *(short *)&d[2];
	short z = *(short *)&d[4];

	if(DEBUG) printf("GYR x=%d, y=%d, z=%d\n\r", x, y, z);

	Est_G.x = x;
	Est_G.y = y;
	Est_G.z = z;

	return 0;
}

int  Get_temp(void)
{
	unsigned char d[2];

    if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), TEMP_OUT_H, 1, &d[1], 1, 1000) != HAL_OK)return -1;
    if (HAL_I2C_Mem_Read(&hi2c1, (DEVID+1), TEMP_OUT_L, 1, &d[0], 1, 1000) != HAL_OK)return -1;

	short tem = *(short *)&d[0];

	if(DEBUG) printf("TEM tem=%d\n\r", tem);

	MPU_temp = tem;

	return 0;
}


int Get_mag(void)
{
	static char pos=0;
	static int timeout = 100;
	unsigned char sr1 = 0;


    unsigned char cTemp = 0x11;
    if (pos == 0){
    	if (HAL_I2C_Mem_Write(&hi2c1, (MAGID+0), AK8963_CNTL, 1, &cTemp, 1, 1000) != HAL_OK)return -1;
		pos++; timeout = 100; return 0;
    }
	//wiringPiI2CWriteReg8(md, AK8963_CNTL, 0x11);
/*	while((sr1&0x01) == 0) {
		HAL_Delay(1);
		//Main_Callback();
		//sr1 = wiringPiI2CReadReg8(md, AK8963_ST1);
	    if (HAL_I2C_Mem_Read(&hi2c1, (MAGID+1), AK8963_ST1, 1, &sr1, 1, 1000) != HAL_OK)return -1;
		if (timeout--<=0)return -1; }
	//usleep(100*1000);
*/
    else if (pos == 1){
    	if (HAL_I2C_Mem_Read(&hi2c1, (MAGID+1), AK8963_ST1, 1, &sr1, 1, 1000) != HAL_OK)return -1;
    	if (timeout--<=0)return -1;
    	if ((sr1&0x01) != 0){pos++;}
    	else return 0;
    }
    else if (pos == 2){
		unsigned char d[6];

		if (HAL_I2C_Mem_Read(&hi2c1, (MAGID+1), AK8963_XOUT_L, 1, &d[0], 1, 1000) != HAL_OK)return -1;
		if (HAL_I2C_Mem_Read(&hi2c1, (MAGID+1), AK8963_XOUT_H, 1, &d[1], 1, 1000) != HAL_OK)return -1;
		if (HAL_I2C_Mem_Read(&hi2c1, (MAGID+1), AK8963_YOUT_L, 1, &d[2], 1, 1000) != HAL_OK)return -1;
		if (HAL_I2C_Mem_Read(&hi2c1, (MAGID+1), AK8963_YOUT_H, 1, &d[3], 1, 1000) != HAL_OK)return -1;
		if (HAL_I2C_Mem_Read(&hi2c1, (MAGID+1), AK8963_ZOUT_L, 1, &d[4], 1, 1000) != HAL_OK)return -1;
		if (HAL_I2C_Mem_Read(&hi2c1, (MAGID+1), AK8963_ZOUT_H, 1, &d[5], 1, 1000) != HAL_OK)return -1;


		short x = *(short *)&d[0];
		short y = *(short *)&d[2];
		short z = *(short *)&d[4];

		x = x*magCorrect_ASA[0];
		y = y*magCorrect_ASA[1];
		z = z*magCorrect_ASA[2];

		unsigned char st2;// = wiringPiI2CReadReg8(md, AK8963_ST2);

		if (HAL_I2C_Mem_Read(&hi2c1, (MAGID+1), AK8963_ST2, 1, &st2, 1, 1000) != HAL_OK)return -1;
		if (!(st2>>4)) return -1; // return if no 16-bit mode

		//y-=150;
		if(DEBUG) printf("MAG x=%d, y=%d, z=%d\n\r", x, y, z);

		Est_M.x = x + MagHardCorrect[0];
		Est_M.y = y + MagHardCorrect[1];
		Est_M.z = z + MagHardCorrect[2];

		pos = 0;
		return 0;
    }
	return 0;
}




unsigned short dig_T1 = 0;
short dig_T2 = 0;
short dig_T3 = 0;
unsigned short dig_P1;
short dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

#define BMP280_S32_t int
#define BMP280_U32_t unsigned int
#define BMP280_S64_t long long

BMP280_S32_t t_fine;
BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T)
{
	BMP280_S32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((BMP280_S32_t)dig_T1<<1))) * ((BMP280_S32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((BMP280_S32_t)dig_T1)) * ((adc_T>>4) - ((BMP280_S32_t)dig_T1))) >> 12) *((BMP280_S32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

BMP280_U32_t bmp280_compensate_P_int64(BMP280_S32_t adc_P)
{
	BMP280_S64_t var1, var2, p;
	var1 = ((BMP280_S64_t)t_fine) - 128000;var2 = var1 * var1 * (BMP280_S64_t)dig_P6;
	var2 = var2 + ((var1*(BMP280_S64_t)dig_P5)<<17);
	var2 = var2 + (((BMP280_S64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (BMP280_S64_t)dig_P3)>>8) + ((var1 * (BMP280_S64_t)dig_P2)<<12);
	var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)dig_P1)>>33;
	if (var1 == 0){return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((BMP280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((BMP280_S64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)dig_P7)<<4);
	return (BMP280_U32_t)p;
}


int BMPInit(void)
{
	unsigned char c[18];
	int i;


	//for (i=0; i<6; i++) c[0+i] = wiringPiI2CReadReg8(bd, (0x88+i));
	for (i=0; i<6; i++) if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), (0x88+i), 1, &c[0+i], 1, 1000) != HAL_OK)return -1;

	dig_T1 = (unsigned short)((c[1]<<8)+c[0]);
	dig_T2 = (signed short)((c[3]<<8)+c[2]);
	dig_T3 = (signed short)((c[5]<<8)+c[4]);

	//for (i=0; i<18; i++) c[i] = wiringPiI2CReadReg8(bd, ((0x8E)+i));
	for (i=0; i<18; i++) if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), ((0x8E)+i), 1, &c[i], 1, 1000) != HAL_OK)return -1;

	dig_P1 = (unsigned short)((c[1]<<8)+c[0]);
	dig_P2 = (signed short)((c[3]<<8)+c[2]);
	dig_P3 = (signed short)((c[5]<<8)+c[4]);
	dig_P4 = (signed short)((c[7]<<8)+c[6]);
	dig_P5 = (signed short)((c[9]<<8)+c[8]);
	dig_P6 = (signed short)((c[11]<<8)+c[10]);
	dig_P7 = (signed short)((c[13]<<8)+c[12]);
	dig_P8 = (signed short)((c[15]<<8)+c[14]);
	dig_P9 = (signed short)((c[17]<<8)+c[16]);

	unsigned char t_sb=0, 	// Controls inactive duration
			filter=0, 		// Controls the time constant of the IIR filter
			spi3w=0;		// SPI interface disable
	unsigned char cTemp = ((t_sb&0x7)<<5) | ((filter&0x7)<<2) | (spi3w&0x1);
	if (HAL_I2C_Mem_Write(&hi2c1, (BMPID+0), 0xF4, 1, &cTemp, 1, 1000) != HAL_OK)return -1;

	return 0;
}


int  GetBmpTemp(void)
{
	unsigned char sr = 1;
	unsigned char cTemp = (0x03+(0x7<<2)+(0x7<<5));		// normal mode
	//wiringPiI2CWriteReg8(bd, 0xF4, (0x03+(0x7<<2)+(0x7<<5)));
	if (HAL_I2C_Mem_Write(&hi2c1, (BMPID+0), 0xF4, 1, &cTemp, 1, 1000) != HAL_OK)return -1;
	//usleep(1000*1000);
	int timeout = 1000;
	while((sr&0x01) != 0) {	// if im_update is running
		HAL_Delay(1);//usleep(1000);
		//Main_Callback();
		//sr = wiringPiI2CReadReg8(bd, 0xF3);
		if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xF3, 1, &sr, 1, 1000) != HAL_OK)return -1;
		if (timeout--<=0)break; }

	unsigned char c[3];
	//c[0] = wiringPiI2CReadReg8(bd, 0xFC);
	//c[1] = wiringPiI2CReadReg8(bd, 0xFB);
	//c[2] = wiringPiI2CReadReg8(bd, 0xFA);
	if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xFC, 1, &c[0], 1, 1000) != HAL_OK)return -1;
	if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xFB, 1, &c[1], 1, 1000) != HAL_OK)return -1;
	if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xFA, 1, &c[2], 1, 1000) != HAL_OK)return -1;
	int tt = (c[0]>>4) + (c[1]<<4) + (c[2]<<12);

	if(DEBUG) printf("T = %d\n\r", bmp280_compensate_T_int32(tt));

	BMP_temp = bmp280_compensate_T_int32(tt);
	return 0;
}


int GetBmpPress(void)
{
	unsigned char sr = 1;
	unsigned char cTemp = (0x03+(0x7<<2)+(0x7<<5));		// normal mode
	//wiringPiI2CWriteReg8(bd, 0xF4, (0x03+(0x7<<2)+(0x7<<5)));
	if (HAL_I2C_Mem_Write(&hi2c1, (BMPID+0), 0xF4, 1, &cTemp, 1, 1000) != HAL_OK)return -1;
	//usleep(1000*1000);
	int timeout = 1000;
	while((sr&0x01) != 0) {	// if im_update is running
		HAL_Delay(1);//usleep(1000);
		//Main_Callback();
		//sr = wiringPiI2CReadReg8(bd, 0xF3);
		if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xF3, 1, &sr, 1, 1000) != HAL_OK)return -1;
		if (timeout--<=0)break; }

	unsigned char c[3];
	//c[0] = wiringPiI2CReadReg8(bd, 0xF9);
	//c[1] = wiringPiI2CReadReg8(bd, 0xF8);
	//c[2] = wiringPiI2CReadReg8(bd, 0xF7);
	if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xF9, 1, &c[0], 1, 1000) != HAL_OK)return -1;
	if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xF8, 1, &c[1], 1, 1000) != HAL_OK)return -1;
	if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xF7, 1, &c[2], 1, 1000) != HAL_OK)return -1;
	int pp = (c[0]>>4) + (c[1]<<4) + (c[2]<<12);

	if(DEBUG) printf("P = %d Pa", bmp280_compensate_P_int64(pp)/256);
	if(DEBUG) printf(" (%d mmHg) \n\r", (bmp280_compensate_P_int64(pp)/256/133));

	BMP_press = bmp280_compensate_P_int64(pp)/256;
	return 0;
}



int GetDataBmp_cycle(void)
{
	static int pos=0;
	static int timeout = 10;
	static int pp =0;
	static int tt =0;

	if (pos == 0){
		unsigned char cTemp = (0x03+(0x7<<2)+(0x7<<5));			// normal mode
		//unsigned char cTemp = (0x01+(0x7<<2)+(0x7<<5));		// forced mode
		if (HAL_I2C_Mem_Write(&hi2c1, (BMPID+0), 0xF4, 1, &cTemp, 1, 1000) != HAL_OK)return -1;
		timeout = 3000/250; // timeout = 3sec. pull every 500ms
		pos++;
	}
	if (pos == 1){
		unsigned char sr;
		if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xF3, 1, &sr, 1, 1000) != HAL_OK)return -1;
		if((sr&0x01) == 1){if (timeout--<=0){pos=0; return -2;}}							//im_update is running
		//else if((sr&(0x01<<3)) == (0x01<<3)){if (timeout--<=0){pos=0; return -2;}}		//measuring
		else pos++;
	}
	if (pos == 2){

		unsigned char c[3];
		if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xF9, 1, &c[0], 1, 1000) != HAL_OK)return -1;
		if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xF8, 1, &c[1], 1, 1000) != HAL_OK)return -1;
		if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xF7, 1, &c[2], 1, 1000) != HAL_OK)return -1;
		pp = (c[0]>>4) + (c[1]<<4) + (c[2]<<12);
		pos++;
	}
	if (pos == 3){
		unsigned char c[3];
		if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xFC, 1, &c[0], 1, 1000) != HAL_OK)return -1;
		if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xFB, 1, &c[1], 1, 1000) != HAL_OK)return -1;
		if (HAL_I2C_Mem_Read(&hi2c1, (BMPID+1), 0xFA, 1, &c[2], 1, 1000) != HAL_OK)return -1;
		tt = (c[0]>>4) + (c[1]<<4) + (c[2]<<12);
		pos++;
	}
	if (pos == 4){
		BMP_press = bmp280_compensate_P_int64(pp)/256;
		pos++;
	}
	if (pos == 5){
		BMP_temp = bmp280_compensate_T_int32(tt);
		pos=0;
		if ((BMP_press==0) && (BMP_temp == 0)) return -2;
	}




	return 0;
}

