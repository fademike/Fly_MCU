/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define PIN_SET_Pin GPIO_PIN_2
#define PIN_SET_GPIO_Port GPIOA
#define SDCARD_SS_Pin GPIO_PIN_12
#define SDCARD_SS_GPIO_Port GPIOB
#define SPI1_NSS_Pin GPIO_PIN_15
#define SPI1_NSS_GPIO_Port GPIOA
#define SI_nIRQ_Pin GPIO_PIN_6
#define SI_nIRQ_GPIO_Port GPIOB
#define SI_SDN_Pin GPIO_PIN_7
#define SI_SDN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

//void Main_Callback(void);

void Printf(const char *fmt, ...);

void RxDataUart(unsigned char data);

#include "MPU9250.h"
#include "diskio.h"
#include "ff.h"

#include <stdint.h>
typedef struct
{
    uint8_t startByte; // $
    uint8_t cmd;// = {0x02,0x2C};
    uint8_t len;

    uint16_t voltage;
    uint16_t current;

    uint8_t mainMotor;	// 0 - off else - on
    uint8_t mainMotorValue;	//0-255
    uint8_t servo1;		// 0 - off else - on
    uint8_t servo1Value;	//0-255
    uint8_t servo2;		// 0 - off else - on
    uint8_t servo2Value;	//0-255
    uint8_t indication;	//0-indication off else - on

    int16_t dofAx;		//data Acceleration
    int16_t dofAy;
    int16_t dofAz;
    int16_t dofGx;		//data Gyroscope
    int16_t dofGy;
    int16_t dofGz;
    int16_t dofMx;		//data Magnitude
    int16_t dofMy;
    int16_t dofMz;

    uint16_t temp1;		// from MPU
    int32_t temp2;		// from BMP
    int32_t press;		// from BMP, in Pa

    uint8_t crc;
    uint8_t endByte;

    //pos_struct qwe;
}CMD2PacketStruct;


typedef struct
{
    uint8_t startByte; // $
    uint8_t cmd;// = {0x02,0x2C};
    uint8_t len;

    uint16_t voltage;
    uint16_t current;

    uint8_t mainMotor;	// 0 - off else - on
    uint8_t mainMotorValue;	//0-255
    uint8_t secondMotor;	// 0 - off else - on
    uint8_t secondMotorValue;	//0-255
    uint8_t servo1;		// 0 - off else - on
    uint8_t servo1Value;	//0-255
    uint8_t servo2;		// 0 - off else - on
    uint8_t servo2Value;	//0-255
    uint8_t indication;	//0-indication off else - on

    int16_t pitch;		//data Acceleration
    int16_t roll;
    int16_t yaw;

    int16_t dofAx;		//data Acceleration
    int16_t dofAy;
    int16_t dofAz;
    int16_t dofGx;		//data Gyroscope
    int16_t dofGy;
    int16_t dofGz;
    int16_t dofMx;		//data Magnitude
    int16_t dofMy;
    int16_t dofMz;

    uint16_t temp1;		// from MPU
    int32_t temp2;		// from BMP
    int32_t press;		// from BMP, in Pa

    uint8_t crc;
    uint8_t endByte;

    //pos_struct qwe;
}CMD4PacketStruct;

#pragma pack(push, 1)

typedef struct	//34+gps(25)
{
    uint8_t startByte; // $
    uint8_t cmd;// = {0x02,0x2C};
    uint8_t len;

    uint8_t indication;	//0-indication off else - on

    uint16_t voltage;
    uint16_t current;

    uint8_t mainMotor;	// 0 - off else - on
    uint8_t mainMotorValue;	//0-255
    uint8_t secondMotor;	// 0 - off else - on
    uint8_t secondMotorValue;	//0-255
    uint8_t servo1;		// 0 - off else - on
    uint8_t servo1Value;	//0-255
    uint8_t servo2;		// 0 - off else - on
    uint8_t servo2Value;	//0-255

    int16_t pitch;		//data Acceleration
    int16_t roll;
    int16_t yaw;


    uint16_t temp1;		// from MPU
    int32_t temp2;		// from BMP
    int32_t press;		// from BMP, in Pa


	uint32_t time;		//GPS Data
	uint32_t day;
	uint32_t longer;
	uint32_t latit;
	uint8_t longer_p;
	uint8_t latit_p;
	uint16_t speed;
	uint16_t hight;
	uint8_t N_sp;
	uint8_t autent;
	uint8_t dir_deg;


    uint8_t crc;
    uint8_t endByte;

    //pos_struct qwe;

    //pos_struct qwe;
}CMD6PacketStruct;
#pragma pack(pop)
/*
struct GPSdataStruct{
	uint32_t time;

	uint32_t day;

	uint16_t speed;
	uint16_t hight;
	uint32_t longer;
	uint8_t longer_p;
	uint32_t latit;
	uint8_t latit_p;
	uint8_t N_sp;
	uint8_t autent;
	uint8_t dir_deg;
}GPSdata;
*/

typedef struct
{
    unsigned char startByte; // $
    unsigned char cmd;// = {0x02,0x2C};

    unsigned char len;

    unsigned char mainMotor;	// 0 - off else - on
    unsigned char mainMotorValue;	//0-255
    unsigned char servo1;		// 0 - off else - on
    unsigned char servo1Value;	//0-255
    unsigned char servo2;		// 0 - off else - on
    unsigned char servo2Value;	//0-255
    unsigned char indication;	//0-indication off else - on

    unsigned char crc;
    unsigned char endByte;
}CMD12PacketStruct;


typedef struct
{
    unsigned char startByte; // $
    unsigned char cmd;// = {0x02,0x2C};

    unsigned char len;

    unsigned char mainMotor;	// 0 - off else - on
    unsigned char mainMotorValue;	//0-255
    unsigned char secondMotor;	// 0 - off else - on
    unsigned char secondMotorValue;	//0-255
    unsigned char servo1;		// 0 - off else - on
    unsigned char servo1Value;	//0-255
    unsigned char servo2;		// 0 - off else - on
    unsigned char servo2Value;	//0-255
    unsigned char indication;	//0-indication off else - on

    unsigned char crc;
    unsigned char endByte;
}CMD14PacketStruct;



typedef struct
{
    unsigned char startByte; // $
    unsigned char cmd;// = {0x02,0x2C};

    unsigned char len;

    unsigned char mainMotor;	// 0 - off else - on
    unsigned char mainMotorValue;	//0-255
    unsigned char secondMotor;	// 0 - off else - on
    unsigned char secondMotorValue;	//0-255
    unsigned char typeManage;		// 0 - simple 1- stabilization
    unsigned char servo1;		// 0 - off else - on
    unsigned char servo1Value;	//0-255
    unsigned char servo2;		// 0 - off else - on
    unsigned char servo2Value;	//0-255
    unsigned char indication;	//0-indication off else - on

    unsigned char StabRoll;
    unsigned char StabPitch;
    unsigned char StabYaw;
    unsigned char StabAngelPitch;
    unsigned char StabAngelRoll;
    unsigned char StabKoeffManage1;	//ROLL
    unsigned char StabKoeffManage2;	//PITCH
    unsigned char StabServo1;		// if 1 - inverse 0 - norm
    unsigned char StabServo2;		// if 1 - inverse 0 - norm

    unsigned char StabKoeffCh1;		//ROLL
    unsigned char StabKoeffCh2;		//PITCH

    unsigned char pos_rev;
    unsigned char pos_ang1;		//0x3;//(0x02;pitch reverse)//
    unsigned char pos_ang2;		//0x13;
    unsigned char StabServoMax1;		// max Value servo 1
    unsigned char StabServoMax2;		// max Value servo 2

    unsigned char CmdTimeout;

    unsigned char math_K_angle;
    unsigned char math_K_bias;
    unsigned char math_K_measure;
    unsigned char math_gyroRate;


    unsigned char crc;
    unsigned char endByte;
}CMD18PacketStruct;


typedef enum
{
	DATAOFF = 0x00U,
	DATAON = 0x01U
} DataOnOff;


/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
