/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

#include "Kalman.h"
#include "MPU9250.h"

#include "Si4463.h"
#include "radio_config_Si4463.h"

#include "uSD.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/



int timerUpdater = 5000;	// Timer for set motors to default state


extern pos_struct Est_A, Est_G, Est_M;
//pos_struct Est_G;
//pos_struct Est_M;
extern signed short MPU_temp;
extern signed int BMP_temp;
extern signed int BMP_press;

float Calc_Pitch=0, Calc_Roll=0, Calc_Yaw=0;	// Calculated position to send out

extern uint8_t aTxBuffer[1];	// DATA from MPU9250 with result value




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


/*
 * Values for set settings SET_...
 */

unsigned char SETmainMotor = 0;	// 0 - off else - on
unsigned char SETmainMotorValue = 0;	//0-255
unsigned char SETsecondMotor = 0;	// 0 - off else - on
unsigned char SETsecondMotorValue = 0;	//0-255
unsigned char SETservo1 = 0;		// 0 - off else - on
unsigned char SETservo1Value = 0;	//0-255
unsigned char SETservo2 = 0;		// 0 - off else - on
unsigned char SETservo2Value = 0;	//0-255
unsigned char SETindication = 0;	//0-indication off else - on

unsigned char SET_typeManage = 0;

unsigned char SET_StabRoll = 0;
unsigned char SET_StabPitch = 0;
unsigned char SET_StabYaw = 0;
unsigned char SET_StabAngelPitch = 0;
unsigned char SET_StabAngelRoll = 0;
unsigned char SET_StabKoeffManage1 = 127;	//ROLL
unsigned char SET_StabKoeffManage2 = 127;	//PITCH
unsigned char SET_StabServo1 = 0;		// if 1 - inverse 0 - norm
unsigned char SET_StabServo2 = 0;		// if 1 - inverse 0 - norm
unsigned char SET_StabKoeffCh1 = 127;		//ROLL
unsigned char SET_StabKoeffCh2 = 127;		//PITCH

unsigned char SET_pos_rev = 0;
unsigned char SET_pos_ang1 = 0x3;//(0x02;pitch reverse)//
unsigned char SET_pos_ang2 = 0x13;

unsigned char SET_StabServoMax1 = 127;		// max Value servo 1
unsigned char SET_StabServoMax2 = 127;		// max Value servo 2

unsigned char SET_CmdTimeout = 20;//200;		// max Value servo 2

unsigned char SET_math_K_angle = 1;		//Kalman Q_angle = SET*0.001;
unsigned char SET_math_K_bias = 3;			//Kalman Q_bias = SET*0.001;
unsigned char SET_math_K_measure = 30;;		//Kalman R_measure = SET*0.001;
unsigned char SET_math_gyroRate = 131;// Kalman rate = gyro/SET_math_gyroRate;


int ADC_data_0 = 0;
int ADC_data_1 = 0;

//circular buffer for receive commands
#define READCMDSTREAM_ARRAY 10

unsigned char ReadCommandStreamArray[READCMDSTREAM_ARRAY][100];
unsigned char ReadCommandStreamArrayCnt[READCMDSTREAM_ARRAY];
int ReadCommandStreamArray_rd=0;
int ReadCommandStreamArray_wr=0;


struct Kalman 	Kalman1 = {0.001, 0.003, 0.03, 0, 0, 0, {{0,0},{0,0}}, {0,0}, 0, 0},	//roll angle stabilization filter
				Kalman2 = {0.001, 0.003, 0.03, 0, 0, 0, {{0,0},{0,0}}, {0,0}, 0, 0};	//pitch angle stabilization filter

int timerMathKalman = 0;	// timer to calculate the angular position of the change in gyroscope


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);
static void MX_SPI1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void ModemControl(void);
int Read_ADC_Channel(int channel);
void MathData(void);
//void SendDataCMD2(void);
//void SendDataCMD4(void);
void SendDataCMD6(void);
void ReadCommandStream(unsigned char data);
void PrepareDataToTx(void);
void ReadStream(void);
void ReadModem(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



int modem_busy = 1;		// modem status to avoid collision (may be to future)

#define PACKET_LEN 64	// length receive buffer of modem

#define SYNCHRO_MODE 1	// 1- synchronization mode; 0- simple mode. More in stm8+si4463 project


#define NONE_P  while(0){};	//GPIO_WriteHigh(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5); Delay_ms(1); GPIO_WriteLow(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5)
//#define NONE_P  Delay_ms(1)

typedef enum  {
	PACKET_NONE 		= 0,
	PACKET_DATA			= 1,
	PACKET_ASK			= 2,
	PACKET_ANSWER		= 3,
} PACKET_TYPE;

//  Variables of modem
//volatile int time_delay=0;
//volatile int time_last_tx=0;
volatile int delay_to_wait_answer=0;
volatile int delay_measure_tx=0;
volatile int delay_synchro=0;
char NeedAnswer=0;
int delay_measure_tx_middle=0;

//DELETE									//circular buffer for send out by RF
#define UART_BUF_SIZE 255			// buffer lenght
char UartRxBuffer[UART_BUF_SIZE];	// buffer
volatile int UartRxBuffer_rd = 0;	// read array
volatile int UartRxBuffer_wr = 0;	//	write array


#define BUFFER_TX_RF 64
char BufferTxRf[UART_BUF_SIZE];	// buffer
volatile int BufferTxRf_len = 0;	// read array



unsigned char rbuff[PACKET_LEN],  wbuff[PACKET_LEN];	// in/out buffer of modem




#define StartUP 2000

typedef enum  {
	thread_MPU_temp 			= 0,
	thread_MPU_acc				= 1,
	thread_MPU_gyro				= 2,
	thread_MPU_ag				= 3,
	thread_MPU_mag				= 4,
	thread_BMP_temp				= 5,
	thread_BMP_press			= 6,
	thread_BMP_all				= 7,
	thread_ADC					= 8,
	thread_ModemControl			= 9,
	thread_PrepareDataToTx		= 10,	// Changed to update data to transfer
	thread_ReadStream			= 11,
	thread_ReInit				= 12,
}thread_name;							// names of my threads


// My threads

struct ThreadFlyStruct {			// This is time settings for some processes
	int name;		//enabled = 1 => on; else off
	int enabled;		//enabled = 1 => on; else off
	int t_counter;		//time to run
	int t_interval;		//interval to run

	int status;			//status thread // if (status<0) => error; else pointer in process
	int priority;		// 1..10; 10 high 1 - low
} 		ThreadFly[] = {	{thread_MPU_temp,			1, 	StartUP, 		500, 	-1, 	1},
						{thread_MPU_acc,			0, 	StartUP, 		300, 	-1, 	1},
						{thread_MPU_gyro,			0, 	StartUP, 		300, 	-1, 	1},
						{thread_MPU_ag,				1, 	StartUP, 		50, 	-1, 	1},
						{thread_MPU_mag,			1, 	StartUP, 		500, 	-1, 	1},
						{thread_BMP_temp,			0, 	StartUP+1000, 	2000, 	-1, 	1},
						{thread_BMP_press,			0, 	StartUP, 		2000, 	-1, 	1},
						{thread_BMP_all,			1, 	StartUP, 		250, 	-1, 	1},
						{thread_ADC,				1, 	StartUP, 		1000, 	0, 		1},
						{thread_ModemControl,		1, 	StartUP, 		100, 	0, 		1},
						{thread_PrepareDataToTx,	1, 	StartUP, 		100, 	0, 		1},	//250
						{thread_ReadStream,			1, 	StartUP+50, 	100, 	0, 		1},
						{thread_ReInit,				1, 	5000, 			5000, 	0, 		1}};


// if i2c interface not disabled, then read/write uSD card by SPI will be not correct
#define EN_I2C SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN) 		//disable I2c
#define DIS_I2C CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN) 	//disable I2c


void moduleMPU9250_Init(void){

	// if any thread is stopped
	if ((ThreadFly[thread_MPU_temp].status >= 0) && ((ThreadFly[thread_MPU_acc].status >= 0) && (ThreadFly[thread_MPU_gyro].status >= 0) &&
		(ThreadFly[thread_MPU_ag].status >= 0) && (ThreadFly[thread_MPU_mag].status >= 0) && (ThreadFly[thread_BMP_temp].status >= 0) &&
		(ThreadFly[thread_BMP_press].status >= 0) && (ThreadFly[thread_BMP_all].status >= 0))) return;


	  EN_I2C;
	  int status_MPU_Init = MPU_Init();						// Init Chip MPU9250
	  DIS_I2C;
	  if (status_MPU_Init == 0) Printf("Device init\n\r");
	  else if (status_MPU_Init == -1) Printf("I2C error, id is %d\n\r", aTxBuffer[0]);
	  else if (status_MPU_Init == -2) Printf("AXIS error, id is %d\n\r", aTxBuffer[0]);
	  else if (status_MPU_Init == -3) Printf("MAG error, id is %d\n\r", aTxBuffer[0]);
	  else if (status_MPU_Init == -4) Printf("BMP error, id is %d\n\r", aTxBuffer[0]);
	  else Printf("Error nothing\n\r");

#define Update(x) if(1){x.status=0; x.t_counter=x.t_interval;}	// thread run

	  if (status_MPU_Init == 0){	// if init MPU9250 OK
		  Update(ThreadFly[thread_MPU_temp]);
		  Update(ThreadFly[thread_MPU_acc]);
		  Update(ThreadFly[thread_MPU_gyro]);
		  Update(ThreadFly[thread_MPU_ag]);
		  Update(ThreadFly[thread_MPU_mag]);
	  }

	  if (status_MPU_Init == 0){EN_I2C;status_MPU_Init = BMPInit();DIS_I2C;}		// Init Chip BMP280

	  if (status_MPU_Init == 0){	//if init MP280 OK
		  Update(ThreadFly[thread_BMP_temp]);
		  Update(ThreadFly[thread_BMP_press]);
		  Update(ThreadFly[thread_BMP_all]);
	  }


}


// handler of all threads
void RunThread(struct ThreadFlyStruct * Struct){
	//if ((Struct->enabled == 0) || (Struct->t_counter != 0) || (Struct->status < 0)) return;
	if (Struct->name == thread_MPU_temp){				EN_I2C;	Struct->status = Get_temp();	DIS_I2C;	}
	else if (Struct->name == thread_MPU_acc){			EN_I2C;	Struct->status = Get_acc();DIS_I2C;}
	else if (Struct->name == thread_MPU_gyro){			EN_I2C;	Struct->status = Get_gyro();	DIS_I2C;}
	else if (Struct->name == thread_MPU_ag){			EN_I2C;	Struct->status = Get_acc(); Get_gyro(); MathData();	DIS_I2C;}
	else if (Struct->name == thread_MPU_mag){			EN_I2C;	Struct->status = Get_mag();	DIS_I2C;}
	else if (Struct->name == thread_BMP_temp){			EN_I2C;	Struct->status = GetBmpTemp();DIS_I2C;}
	else if (Struct->name == thread_BMP_press){			EN_I2C;	Struct->status = GetBmpPress();	DIS_I2C;}
	else if (Struct->name == thread_BMP_all){EN_I2C;	Struct->status = GetDataBmp_cycle();	DIS_I2C;}
	else if (Struct->name == thread_ADC){				ADC_data_0 = Read_ADC_Channel(0); ADC_data_0=(ADC_data_0*3300)/0xFFF;}
	else if (Struct->name == thread_ModemControl){		ModemControl();	}
	else if (Struct->name == thread_PrepareDataToTx){	PrepareDataToTx();}
	else if (Struct->name == thread_ReadStream){		ReadStream();}
	else if (Struct->name == thread_ReInit){			moduleMPU9250_Init();}

	//Struct->t_counter = Struct->t_interval;	// Update counter
}



//poll all threads
//execute the stream with the highest priority and exit
void Thread_Cycle(void)
{
	int thread_all=sizeof(ThreadFly)/sizeof(ThreadFly[0]);
	int priority=10;
	int priorityVar[11] = {0,};	// stories priority options

	//Look at all the priorities
	for (priority=10;priority>0; priority--){	// looking for what priority options exist.
		int cnt = 0;
		priorityVar[priority]=0; 	// priority does not exist
		for (cnt=0; cnt<thread_all; cnt++){
			if (ThreadFly[cnt].priority == priority)
			{
				priorityVar[priority]=1; // priority exist
				continue;
			}
		}
	}
	//Look at all exist the priorities
	for (priority=10;priority>0; priority--){
		if (priorityVar[priority] != 1) continue;	// priority does not exist
		int cnt = 0;
		for (cnt=0; cnt<thread_all; cnt++){
			if ((ThreadFly[cnt].priority == priority) && (ThreadFly[cnt].enabled == 1) && (ThreadFly[cnt].t_counter == 0) && (ThreadFly[cnt].status >= 0)){
				RunThread(&ThreadFly[cnt]);	// Run thread
				ThreadFly[cnt].t_counter = ThreadFly[cnt].t_interval;	// Update counter
				return;
			}
		}
	}
}



// 1ms SYS Callback
void HAL_SYSTICK_Callback(void)
{
	static int timSec = 0;
	if (0)if (timSec++>5000){timSec=0; Printf("rx time is %d\n\r", GPSdata.time);
	 Printf("day is %d\n\r", GPSdata.day);
	 Printf("autent is %d\n\r", GPSdata.autent);
	 Printf("speed is %d\n\r", GPSdata.speed);
	 Printf("dir_deg is %d\n\r", GPSdata.dir_deg);
	 Printf("longer is %d\n\r", GPSdata.longer);
	 Printf("longer_p is %d\n\r", GPSdata.longer_p);
	 Printf("latit is %d\n\r", GPSdata.latit);
	 Printf("latit_p is %d\n\r", GPSdata.latit_p);
	 Printf("N_sp is %d\n\r", GPSdata.N_sp);
	 Printf("hight is %d\n\r", GPSdata.hight);
	}

	timerMathKalman++;

	if(delay_to_wait_answer>0) delay_to_wait_answer--;
	else ThreadFly[thread_ModemControl].t_counter = 0;//{thread_ModemControl.t_counter = 0;}														//TODO


	if(delay_synchro>0) delay_synchro--;	// timer for synchronization modem
	delay_measure_tx++;

// Updater time for counters
	if (ThreadFly[thread_MPU_temp].t_counter>0) ThreadFly[thread_MPU_temp].t_counter--;
	if (ThreadFly[thread_MPU_acc].t_counter>0) ThreadFly[thread_MPU_acc].t_counter--;
	if (ThreadFly[thread_MPU_gyro].t_counter>0) ThreadFly[thread_MPU_gyro].t_counter--;
	if (ThreadFly[thread_MPU_ag].t_counter>0) ThreadFly[thread_MPU_ag].t_counter--;
	if (ThreadFly[thread_MPU_mag].t_counter>0) ThreadFly[thread_MPU_mag].t_counter--;
	if (ThreadFly[thread_BMP_temp].t_counter>0) ThreadFly[thread_BMP_temp].t_counter--;
	if (ThreadFly[thread_BMP_press].t_counter>0) ThreadFly[thread_BMP_press].t_counter--;
	if (ThreadFly[thread_BMP_all].t_counter>0) ThreadFly[thread_BMP_all].t_counter--;
	if (ThreadFly[thread_ADC].t_counter>0) ThreadFly[thread_ADC].t_counter--;
	if (ThreadFly[thread_ModemControl].t_counter>0) ThreadFly[thread_ModemControl].t_counter--;
	if (ThreadFly[thread_PrepareDataToTx].t_counter>0) ThreadFly[thread_PrepareDataToTx].t_counter--;
	if (ThreadFly[thread_ReadStream].t_counter>0) ThreadFly[thread_ReadStream].t_counter--;
	if (ThreadFly[thread_ReInit].t_counter>0) ThreadFly[thread_ReInit].t_counter--;


	if (timerUpdater>0){timerUpdater--;}
	else {
		if (SET_typeManage==0) {		// manual manage
			timerUpdater=SET_CmdTimeout*100;
			  TIM3->CCR1 = 1000;	//main motor
			  TIM3->CCR2 = 1500;	//servo2
			  TIM3->CCR3 = 1500;	//servo1
			  TIM3->CCR4 = 1000;	//second motor
		}
		else if (SET_typeManage==1) {	//auto manage
			timerUpdater=SET_CmdTimeout*100;
			  TIM3->CCR1 = 1000;	//main motor
			  TIM3->CCR4 = 1000;	//second motor
			SET_StabRoll=0;
			SET_StabPitch=0;
			SET_StabYaw=0;
		}

	}

}

/* ADC init function */
static void ADC_Change_Channel(int Channel)
{

	  ADC_ChannelConfTypeDef sConfig;

	    /**Common config
	    */
	  hadc1.Instance = ADC1;
	  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	  hadc1.Init.ContinuousConvMode = DISABLE;
	  hadc1.Init.DiscontinuousConvMode = DISABLE;
	  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.NbrOfConversion = 1;
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	    /**Configure Regular Channel
	    */



    /**Configure for the selected ADC regular channel to be converted.
    */

	  sConfig.Channel = Channel;//ADC_CHANNEL_1;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

}



int Read_ADC_Channel(int channel)
{
	int adcResult=0, i=0,Count = 10;	// if Count = 1 => Time read = 50us; if Count = 10 => Time read = 63.6us
	  HAL_ADC_DeInit(&hadc1);
	  ADC_Change_Channel(channel);


	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 100);
	  for (i=0; i<Count; i++)
		  {
		  adcResult += HAL_ADC_GetValue(&hadc1);
		  }
	  HAL_ADC_Stop(&hadc1);
	  return (adcResult/Count);
}



#include <stdarg.h>

void Printf(const char *fmt, ...)
{
	static char buf[256];
	char *p;
	va_list lst;

	va_start(lst, fmt);
	vsprintf(buf, fmt, lst);
	va_end(lst);

	p = buf;
	while(*p) {
		HAL_UART_Transmit(&huart1, (unsigned char *)p, 1, 500);
		p++;
	}
}




void PrepareDataToTx(void){
//	if (1)if (SendDataUART_fl){
//		SendDataUART_fl=0;
	SendDataCMD6();
		//HAL_UART_Transmit(&huart1, PacketToSend, sizeof(CMD2PacketStruct)-2, 500);	//41


//	}
}

void ReadStream(void){
	if (ReadCommandStreamArray_rd != ReadCommandStreamArray_wr){

		int cnt = ReadCommandStreamArrayCnt[ReadCommandStreamArray_rd];
		int i=0;
	  if (cnt>0)do{
		 ReadCommandStream(ReadCommandStreamArray[ReadCommandStreamArray_rd][i++]);
	  }while(--cnt>0);
	  if(++ReadCommandStreamArray_rd>=READCMDSTREAM_ARRAY)ReadCommandStreamArray_rd=0;

	}
}



#include "Kalman.h"
#include <math.h>

void MathData(void)		//Calculate data by acceleration, gyroscope and magnetometer
{
	#define RAD_TO_DEG (180/3.14159265)
	// type 1	// true
	//double kalAngle1 = getAngle1((atan2(Est_A.y,Est_A.z))*RAD_TO_DEG, (double)Est_G.x/131.0, (double)(timerMathKalman)/1000);
	//double kalAngle2 = getAngle2((atan2(Est_A.x,Est_A.z))*RAD_TO_DEG, -((double)Est_G.y/131.0), (double)(timerMathKalman)/1000);

	// type 2	face to up dlo is back
	//double kalAngle1 = getAngle2((-atan2(-Est_A.x,Est_A.z))*RAD_TO_DEG, -((double)Est_G.y/131.0), (double)(timerMathKalman)/1000);
	//double kalAngle2 = getAngle1((-atan2(Est_A.y,Est_A.z))*RAD_TO_DEG, -(double)Est_G.x/131.0, (double)(timerMathKalman)/1000);

	// type 3 ( revers type 2 by roll 180)	face to down dlo is back

	//double kalAngle2 = getAngle1((atan2(-Est_A.y,-Est_A.z))*RAD_TO_DEG, (double)Est_G.x/131.0, (double)(timerMathKalman)/1000);
	//double kalAngle1 = getAngle2((atan2(-Est_A.x,-Est_A.z))*RAD_TO_DEG, -((double)Est_G.y/131.0), (double)(timerMathKalman)/1000);



	double ang1;// = (atan2(-Est_A.y,-Est_A.z))*RAD_TO_DEG;
	double gyro_div = SET_math_gyroRate;
	double gyro1 = (double)Est_G.x/gyro_div;//131.0;

	signed short arg1_1 = Est_A.y;
	signed short arg1_2 = Est_A.z;

	if ((SET_pos_ang1&(0x1<<0)) != 0) arg1_1 = -arg1_1;
	if ((SET_pos_ang1&(0x1<<1)) != 0) arg1_2 = -arg1_2;

	ang1 = (atan2(arg1_1,arg1_2))*RAD_TO_DEG;
	if ((SET_pos_ang1&(0x1<<5)) != 0) ang1 = -ang1;
	if ((SET_pos_ang1&(0x1<<4)) != 0) gyro1 = -gyro1;

	double ang2;// = (atan2(-Est_A.x,-Est_A.z))*RAD_TO_DEG;
	double gyro2 = ((double)Est_G.y/gyro_div);//131.0);

	signed short arg2_1 = Est_A.x;
	signed short arg2_2 = Est_A.z;

	if ((SET_pos_ang2&(0x1<<0)) != 0) arg2_1 = -arg2_1;
	if ((SET_pos_ang2&(0x1<<1)) != 0) arg2_2 = -arg2_2;

	ang2 = (atan2(arg2_1,arg2_2))*RAD_TO_DEG;
	if ((SET_pos_ang2&(0x1<<5)) != 0) ang2 = -ang2;
	if ((SET_pos_ang2&(0x1<<4)) != 0) gyro2 = -gyro2;

	ang1+=180;	//convert angle
	ang2+=180;

	double kalAngle2 = getAngle(&Kalman1, ang1, gyro1, (double)(timerMathKalman)/1000);
	double kalAngle1 = getAngle(&Kalman2, ang2, gyro2, (double)(timerMathKalman)/1000);

	if (SET_pos_rev != 0){
		double temp =  kalAngle2;
		kalAngle2 = kalAngle1;
		kalAngle1 = temp;
	}


	kalAngle1-=180;
	kalAngle2-=180;
	timerMathKalman=0;

	Calc_Roll = kalAngle1;
	Calc_Pitch = kalAngle2;
	Calc_Yaw= atan2(Calc_Pitch, Calc_Roll)*RAD_TO_DEG;



	//stabilization	//FIXME

if (SET_typeManage==0) return;		// if manual control

//if auto correction angle by position


	/*
	 *
unsigned char SET_typeManage = 0;
unsigned char SET_StabRoll = 0;
unsigned char SET_StabPitch = 0;
unsigned char SET_StabYaw = 0;
unsigned char SET_StabAngelRoll = 0;
unsigned char SET_StabAngelPitch = 0;
unsigned char SET_StabMaxValue1 = 127;	//ROLL
unsigned char SET_StabMaxValue2 = 127;	//PITCH
unsigned char SET_StabServo1 = 0;		// if 1 - inverse 0 - norm
unsigned char SET_StabServo2 = 0;		// if 1 - inverse 0 - norm
unsigned char SET_StabKoeffCh1 = 90;		//ROLL
unsigned char SET_StabKoeffCh2 = 90;		//PITCH

unsigned char SET_pos_rev = 0;
unsigned char SET_pos_ang1 = 0x3;//(0x02;pitch reverse)//
unsigned char SET_pos_ang2 = 0x13;

unsigned char SET_StabServoMax1 = 127;		// max Value servo 1
unsigned char SET_StabServoMax2 = 127;		// max Value servo 2

	 */

	int CH_pitch = 0;
	int CH_roll = 0;

	signed char SetAngel = *(char *)&SET_StabAngelPitch;
	signed char SetRollAngel = *(char *)&SET_StabAngelRoll;

	CH_roll = *(signed char *)&SET_StabRoll;
	CH_pitch = *(signed char *)&SET_StabPitch;

	int servo1 = 0;
	int servo2 = 0;

	//int diff = ((Calc_Roll + CH_roll)*SET_StabMaxValue1)/SET_StabKoeffCh1;
	int diff = (((Calc_Roll + SetRollAngel)*SET_StabKoeffCh1)/45) + ((CH_roll*SET_StabKoeffManage1)/100);
	//int position = ((Calc_Pitch + SetAngel+ CH_pitch)*SET_StabMaxValue2)/SET_StabKoeffCh2;
	int position = ((Calc_Pitch + SetAngel)*SET_StabKoeffCh2)/45 +
											(CH_pitch*SET_StabKoeffManage2)/100;

	int diff1 = 0;
	int diff2 = 0;

	if ((SET_StabServo1&0x2) == 0)	{diff1 = diff;}
	else	{diff1 = -diff;}
	if ((SET_StabServo2&0x2) == 0)	{diff2 = diff;}
	else	{diff2 = -diff;}


	if ((SET_StabServo1&0x1) == 0)	{servo1 = position +diff1;}	//norm value
	else  							{servo1 = -position +diff1;}//inverse value

	if ((SET_StabServo2&0x1) == 0)	{servo2 = -position +diff2;}//norm value
	else 							{servo2 = position +diff2;}	//inverse value


//#define SERVO_MAXVALUE 127
	if (servo1>SET_StabServoMax1) servo1 = SET_StabServoMax1;
	if (servo1<(0-SET_StabServoMax1)) servo1 = -SET_StabServoMax1;
	if (servo2>SET_StabServoMax2) servo2 = SET_StabServoMax2;
	if (servo2<(0-SET_StabServoMax2)) servo2 = -SET_StabServoMax2;


	int setValue;

	setValue = 1500 + ((servo1*500)/0x7F);
	if ((setValue>2000) || (setValue<1000)) setValue = 1500;
	TIM3->CCR2 = setValue;

	setValue = 1500 + ((servo2*500)/0x7F);
	if ((setValue>2000) || (setValue<1000)) setValue = 1500;
	TIM3->CCR3 = setValue;


}


int GPRMC = 0;
char GPRMC_s[5] = "GPRMC";
int GPGGA = 0;
char GPGGA_s[5] = "GPGGA";

void RxDataUart(unsigned char data){		// read GPS data

	static int pos = 0;
	static int dataPos = 0;
	static int dataRead = 0;
	static int dataReadPoint = 0;
	static char dataReadChar = 0;

	if ((pos == 0) && (data == '$')) {pos++; GPRMC=0; GPGGA=0; dataPos=0; dataReadChar='\0';}
	else if ((pos >= 1) && (pos <= 5)){
		if (GPRMC_s[pos-1] == data) GPRMC++;
		if (GPGGA_s[pos-1] == data) GPGGA++;
		pos++;
	}
	else if ((pos == 6) && ((GPRMC == 5) || GPGGA == 5)) {//datarx++;
		if (data == ','){
			if (GPRMC == 5){
				if (dataPos == 1){GPSdata.time = dataRead/100;}
				if (dataPos == 2){GPSdata.autent = dataReadChar;}
				if (dataPos == 7){GPSdata.speed = dataRead/dataReadPoint;}
				if (dataPos == 8){GPSdata.dir_deg = dataRead/dataReadPoint;}
				if (dataPos == 9){GPSdata.day = dataRead;}
			}
			else if (GPGGA == 5){
				if (dataPos == 1){GPSdata.time = dataRead/100;}
				if (dataPos == 2){GPSdata.longer = dataRead;}
				if (dataPos == 3){GPSdata.longer_p = dataReadChar;}
				if (dataPos == 4){GPSdata.latit = dataRead;}
				if (dataPos == 5){GPSdata.latit_p = dataReadChar;}
				if (dataPos == 7){GPSdata.N_sp = dataRead;}
				if (dataPos == 9){GPSdata.hight = dataRead/(dataReadPoint);}
			}
			dataRead=0; dataReadPoint=0;
			dataPos++;}

		else if ((data == '\n') || (data == '\r') || (data < 0x10)) pos=0;
		else {
			if ((data>=0x30) && (data<=0x39)) {dataRead*=10; dataRead+=data-0x30; if (dataReadPoint!=0)dataReadPoint*=10;}
			else if (data == '.') dataReadPoint++;
			else dataReadChar += data;
		}
	}
	else pos=0;


}





/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */

  CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN); //disable I2c

  HAL_IWDG_Init(&hiwdg);
  __HAL_IWDG_START(&hiwdg);
  HAL_IWDG_Refresh(&hiwdg);

  HAL_TIM_Base_Start(&htim3);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  TIM3->CCR1 = 1000;	//main motor
  TIM3->CCR2 = 1500;	//servo2
  TIM3->CCR3 = 1500;	//servo1
  TIM3->CCR4 = 1000;	//second motor


  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);	// Enable Interrupt



  //uSD_Test();

  moduleMPU9250_Init();

  HAL_IWDG_Refresh(&hiwdg);

  HAL_GPIO_WritePin(SI_SDN_GPIO_Port, SI_SDN_Pin, GPIO_PIN_SET);	//Shut down on (si4463)
  HAL_Delay(200);
  HAL_GPIO_WritePin(SI_SDN_GPIO_Port, SI_SDN_Pin, GPIO_PIN_RESET);	//Shut down off (si4463)
  HAL_Delay(200);


  unsigned char buffer[16];
  do{

	HAL_IWDG_Refresh(&hiwdg);

  	SI446X_PART_INFO(buffer);

  	Printf("Si data is : 0x%x, 0x%x\n\r", buffer[2], buffer[3]);
  	if ((buffer[2] == 0x44) && (buffer[3]== 0x63)) Printf("Dev is si4463\n\r");
  	HAL_Delay(100);

  }while((buffer[3]!= 0x63));


  HAL_IWDG_Refresh(&hiwdg);

  RFinit();			//Set Settings Config to Si4463

  SI446X_FIFOINFO(0, 0, 1, 1);		// BUFFER CLEAR

  changeState(STATE_RX);			// Set state to RX

  HAL_IWDG_Refresh(&hiwdg);

Printf("Started...\n\r");







  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_IWDG_Refresh(&hiwdg);

	  Thread_Cycle();

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 1250;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PIN_SET_Pin|SPI1_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SDCARD_SS_Pin|SI_SDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PIN_SET_Pin SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = PIN_SET_Pin|SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SDCARD_SS_Pin */
  GPIO_InitStruct.Pin = SDCARD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SDCARD_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SI_nIRQ_Pin */
  GPIO_InitStruct.Pin = SI_nIRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SI_nIRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SI_SDN_Pin */
  GPIO_InitStruct.Pin = SI_SDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SI_SDN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void SendDataCMD6(void)	// cmd to send data out by RF
{
	unsigned char PacketToSend[64];	// buffer to send out by RF
	  CMD6PacketStruct * ptr = (CMD6PacketStruct *)&PacketToSend[0];

	  ptr->startByte = '$';
	  ptr->cmd = 0x6;
	  ptr->len = sizeof(CMD6PacketStruct)-3-2;
	  ptr->voltage = ADC_data_0&0xFFFF;
	  ptr->current = ADC_data_1&0xFFFF;

	  ptr->mainMotor = SETmainMotor;
	  ptr->mainMotorValue = SETmainMotorValue;
	  ptr->secondMotor = SETsecondMotor;
	  ptr->secondMotorValue = SETsecondMotorValue;
	  ptr->servo1 = SETservo1;
	  ptr->servo1Value = SETservo1Value;
	  ptr->servo2 = SETservo2;
	  ptr->servo2Value = SETservo2Value;
	  ptr->indication = SETindication;

	  ptr->time = GPSdata.time;
	  ptr->day = GPSdata.day;
	  ptr->speed = GPSdata.speed;
	  ptr->hight = GPSdata.hight;
	  ptr->longer = GPSdata.longer;
	  ptr->longer_p = GPSdata.longer_p;
	  ptr->latit = GPSdata.latit;
	  ptr->latit_p = GPSdata.latit_p;
	  ptr->N_sp = GPSdata.N_sp;
	  ptr->autent = GPSdata.autent;
	  ptr->dir_deg = GPSdata.dir_deg;

	  ptr->temp1 = MPU_temp;
	  ptr->temp2 = BMP_temp;

	  ptr->pitch = Calc_Pitch*100;
	  ptr->roll = Calc_Roll*100;
	  ptr->yaw = Calc_Yaw*100;


	  ptr->press = BMP_press;
	  ptr->crc = 0x00;
	  ptr->endByte = '#';


	  unsigned char * pptr = &PacketToSend[3];
	  int i=0;
	  unsigned char matchCRC = 0;
	  for (i=0;i<(sizeof(CMD6PacketStruct)-3-2); i++)matchCRC ^= *pptr++;

	  ptr->crc = matchCRC;

	  BufferTxRf_len = 60;//sizeof(CMD6PacketStruct);
	  int r=0;
	  for (r=0; r<(sizeof(CMD6PacketStruct)); r++){
		  BufferTxRf[r] = PacketToSend[r];

		  /*
	  	UartRxBuffer[UartRxBuffer_wr] = PacketToSend[r];
	  	//if (UartRxBuffer_wr != UartRxBuffer_rd){	}						// if have not data to send => update packet to send.)
	  	//else {
	  		if (++UartRxBuffer_wr>=UART_BUF_SIZE) UartRxBuffer_wr = 0;*/
	  	//}	// or add command to send
	  }
}

void UseRxData()
{

	if (SETmainMotor != 0) TIM3->CCR1 = 1000 + ((SETmainMotorValue*1000)/0xFF);
	else TIM3->CCR1 = 1000;

	if (SETsecondMotor != 0) TIM3->CCR4 = 1000 + ((SETsecondMotorValue*1000)/0xFF);
	else TIM3->CCR4 = 1000;
if (SET_typeManage==0){	// manual manage
	//char cTemp1 = *(char *)&SETservo1Value;
	if (SETservo1 != 0) {
		int setValue;
		signed char scTemp = *(signed char *)&SETservo1Value;
		//if (SETservo1Value<=0x80) setValue = 1500 + ((SETservo1Value*500)/0x80);
		//else { setValue = 1500 - ((SETservo1Value*500)/0x80);}
		setValue = 1500 + ((scTemp*500)/0x7F);
		if ((setValue>2000) || (setValue<1000)) setValue = 1500;
		TIM3->CCR2 = setValue;
	}
	else TIM3->CCR2 = 1500;

	if (SETservo2 != 0) {
		int setValue;
		signed char scTemp = *(signed char *)&SETservo2Value;
		//if (SETservo2Value<=0x80) setValue = 1500 + ((SETservo2Value*500)/0x80);
		//else { setValue = 1500 - ((SETservo2Value*500)/0x80);}	//SETservo2Value = (~SETservo2Value)+1;
		setValue = 1500 + ((scTemp*500)/0x7F);
		if ((setValue>2000) || (setValue<1000)) setValue = 1500;
		TIM3->CCR3 = setValue;
	}
	else TIM3->CCR3 = 1500;
}
/*																//FIXME
	int r=0;
	for (r=0; r<(sizeof(CMD6PacketStruct)-2); r++){
		UartRxBuffer[UartRxBuffer_wr] = PacketToSend[r];
		if (++UartRxBuffer_wr>=UART_BUF_SIZE) UartRxBuffer_wr = 0;
	}*/
}



void ReadCommandStream(unsigned char data)
{
	static unsigned char cmd=0;

	static int position = 0;
	static int data_len = 0;

	static unsigned char PacketRx[255];
	static int PacketRxB = 0;
	static unsigned char PacketCRC = 0;

	unsigned char cTemp = data;//(uint8_t)(huart1.Instance->DR & (uint8_t)0x00FF);
	//HAL_UART_Transmit(&huart1, &UART_rx_bufer[UART_rx0_index], 1, 100);
	//HAL_UART_Receive

	PacketCRC ^= cTemp;

	if ((position == 0) && (cTemp == '$')) {position++; PacketCRC = 0;}
	else if ((position == 1) && ((cTemp == 0x12) || (cTemp == 0x14) || (cTemp == 0x16) || (cTemp == 0x18))) {cmd = cTemp; position++;}

	else if (position == 2) {data_len = cTemp; position++; PacketRxB = 0; PacketCRC = 0;}
	else if ((position == 3) && (data_len > 0)) {PacketRx[3+PacketRxB++] = cTemp; data_len--;}
	else if ((position == 3) && (data_len <= 0)) {position++;} //Check CRC


	else if ((position == 4) && (cTemp == '#')){

		timerUpdater=SET_CmdTimeout*100;
		//Printf("Motorv= %d\n\r", PacketRx[1]);
/*
		if (cmd == 0x12){
			SETmainMotor = PacketRx[0];
			SETmainMotorValue = PacketRx[1];
			SETservo1 = PacketRx[2];
			SETservo1Value = PacketRx[3];
			SETservo2 = PacketRx[4];
			SETservo2Value = PacketRx[5];
			SETindication = PacketRx[6];
		}
		else if (cmd == 0x14){SET_typeManage=0;
			SETmainMotor = PacketRx[2];
			SETmainMotorValue = PacketRx[3];
			SETsecondMotor = PacketRx[0];
			SETsecondMotorValue = PacketRx[1];
			SETservo1 = PacketRx[2+2];
			SETservo1Value = PacketRx[3+2];
			SETservo2 = PacketRx[4+2];
			SETservo2Value = PacketRx[5+2];
			SETindication = PacketRx[6+2];
		}
		else*/ if ((cmd == 0x16) || (cmd == 0x18)){

			CMD18PacketStruct * ptr = (CMD18PacketStruct *)PacketRx;

			SETmainMotor = ptr->mainMotor;					//PacketRx[2];
			SETmainMotorValue = ptr->mainMotorValue;		//PacketRx[3];
			SETsecondMotor = ptr->secondMotor;				//PacketRx[0];
			SETsecondMotorValue = ptr->secondMotorValue;	//PacketRx[1];
			SETservo1 = ptr->servo1;						//PacketRx[2+2+1];
			SETservo1Value = ptr->servo1Value;				//PacketRx[3+2+1];
			SETservo2 = ptr->servo2;						//PacketRx[4+2+1];
			SETservo2Value = ptr->servo2Value;				//PacketRx[5+2+1];
			SETindication = ptr->indication;				//PacketRx[6+2+1];

			SET_typeManage = ptr->typeManage;				//PacketRx[2+2];
			SET_StabRoll = ptr->StabRoll;					//PacketRx[10];
			SET_StabPitch = ptr->StabPitch;					//PacketRx[10+1];
			SET_StabYaw = ptr->StabYaw;						//PacketRx[10+2];
			SET_StabAngelPitch = ptr->StabAngelPitch;		//PacketRx[10+3];
			SET_StabAngelRoll = ptr->StabAngelRoll;			//PacketRx[10+4];
			SET_StabKoeffManage1 = ptr->StabKoeffManage1;	//PacketRx[10+5];
			SET_StabKoeffManage2 = ptr->StabKoeffManage2;	//PacketRx[10+6];
			SET_StabServo1 = ptr->StabServo1;				//PacketRx[10+7];
			SET_StabServo2 = ptr->StabServo2;				//PacketRx[10+8];
			SET_StabKoeffCh1 = ptr->StabKoeffCh1;			//PacketRx[10+9];
			SET_StabKoeffCh2 = ptr->StabKoeffCh2;			//PacketRx[10+10];

			SET_pos_rev = ptr->pos_rev;						//PacketRx[10+11];
			SET_pos_ang1 = ptr->pos_ang1;					//PacketRx[10+12];
			SET_pos_ang2 = ptr->pos_ang2;					//PacketRx[10+13];

			SET_StabServoMax1 = ptr->StabServoMax1;			//PacketRx[10+14];
			SET_StabServoMax2 = ptr->StabServoMax2;			//PacketRx[10+15];

			SET_CmdTimeout = ptr->CmdTimeout;				//PacketRx[10+16];


			if (cmd == 0x18){

				SET_math_K_angle = ptr->math_K_angle;		//PacketRx[10+17];
				SET_math_K_bias = ptr->math_K_bias;			//PacketRx[10+18];
				SET_math_K_measure = ptr->math_K_measure;	//PacketRx[10+19];
				SET_math_gyroRate = ptr->math_gyroRate;		//PacketRx[10+20];

				//SET_param(SET_math_K_angle, SET_math_K_bias, SET_math_K_measure);

				Kalman1.Q_angle = 0.001*SET_math_K_angle;
				Kalman1.Q_bias = 0.001*SET_math_K_bias;
				Kalman1.R_measure = 0.001*SET_math_K_measure;

				Kalman2.Q_angle = 0.001*SET_math_K_angle;
				Kalman2.Q_bias = 0.001*SET_math_K_bias;
				Kalman2.R_measure = 0.001*SET_math_K_measure;

			}

		}

		UseRxData();

		position = 0;

	}
	else position = 0;


}

#define MODEM_PINCONTROL 1	// For debug pin output




void ReadModem(void)
{
	if (modem_busy!=0)return;

    uint8_t xStatus = getStatus();
    if ((xStatus&(1<<4))!=0){	                                                //PACKET_RX

      //if ((xStatus&(1<<3))!=0){GPIO_WriteHigh(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);Delay_ms(1);GPIO_WriteLow(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);changeState(STATE_RX); continue;}

            RFread(rbuff, PACKET_LEN);
#if SYNCHRO_MODE
            if (rbuff[0] == PACKET_NONE ){
              NONE_P;
              NeedAnswer = 0; delay_to_wait_answer = 0;
            }
            else if (rbuff[0] == PACKET_ASK ){
              NONE_P;
              NeedAnswer = PACKET_ANSWER; delay_to_wait_answer = 0;
            }
            else if (rbuff[0] == PACKET_ANSWER ){
              NONE_P;
              NeedAnswer = 0; delay_to_wait_answer = 0;
            }
            else
#endif
              if (rbuff[0] == PACKET_DATA){

              NONE_P;

              NeedAnswer = PACKET_ANSWER; delay_to_wait_answer = 0;
              char cnt = rbuff[1]&0x3F;
              int i=2;
              if (cnt >=1){
            	  ReadCommandStreamArrayCnt[ReadCommandStreamArray_wr] = cnt;
                do{
               	 //HAL_UART_Transmit(&huart1, &rbuff[i], 1, 100);
               	// ReadCommandStream(rbuff[i]);
               	ReadCommandStreamArray[ReadCommandStreamArray_wr][i-2] = rbuff[i];
               	 i++;
                  //while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
                  //UART1_SendData8((uint8_t)rbuff[i++]);
                }while(--cnt>0);
                if(++ReadCommandStreamArray_wr>=READCMDSTREAM_ARRAY)ReadCommandStreamArray_wr=0;
              }
            }
            changeState(STATE_RX);

            if (MODEM_PINCONTROL)HAL_GPIO_WritePin(PIN_SET_GPIO_Port, PIN_SET_Pin, GPIO_PIN_RESET);
            return;//continue;
    }
}






void ModemControl(void)
{
if (MODEM_PINCONTROL)HAL_GPIO_WritePin(PIN_SET_GPIO_Port, PIN_SET_Pin, GPIO_PIN_SET);
modem_busy=1;
    //uint8_t xStatus = getStatus();

    uint8_t buffer[6];
    SI446X_INT_STATUS(buffer);

    uint8_t xStatus = buffer[3];
    if(0)if ((xStatus&(1<<4))==0){	// GET carrier!?!? defect! need to clear bit.
    	if ((buffer[6]&0x3) != 0) {ThreadFly[thread_ModemControl].t_counter=0; return;}//thread_ModemControl.t_counter = 0; return;}
    }
     if ((xStatus&(1<<4))!=0){	                                                //PACKET_RX

       //if ((xStatus&(1<<3))!=0){GPIO_WriteHigh(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);Delay_ms(1);GPIO_WriteLow(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);changeState(STATE_RX); continue;}

             RFread(rbuff, PACKET_LEN);
             SI446X_FIFOINFO(0, 0, 1, 1);		// BUFFER CLEAR
#if SYNCHRO_MODE
             if (rbuff[0] == PACKET_NONE ){
               NONE_P;
               NeedAnswer = 0; delay_to_wait_answer = 0;
             }
             else if (rbuff[0] == PACKET_ASK ){
               NONE_P;
               NeedAnswer = PACKET_ANSWER; delay_to_wait_answer = 0;
             }
             else if (rbuff[0] == PACKET_ANSWER ){
               NONE_P;
               NeedAnswer = 0; delay_to_wait_answer = 0;
             }
             else
#endif
               if (rbuff[0] == PACKET_DATA){

               NONE_P;

               NeedAnswer = PACKET_ANSWER; delay_to_wait_answer = 0;
               char cnt = rbuff[1]&0x3F;
               int i=2;
               if (cnt >=1){
                 do{
                	 //HAL_UART_Transmit(&huart1, &rbuff[i], 1, 100);
                	 ReadCommandStream(rbuff[i++]);
                	 //i++;
                   //while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
                   //UART1_SendData8((uint8_t)rbuff[i++]);
                 }while(--cnt>0);
               }
             }
             changeState(STATE_RX);

             if (MODEM_PINCONTROL)HAL_GPIO_WritePin(PIN_SET_GPIO_Port, PIN_SET_Pin, GPIO_PIN_RESET);
             modem_busy=0;
             //thread_ModemControl.t_counter = 0;
             ThreadFly[thread_ModemControl].t_counter=0;
             return;//continue;
     }

#if 0
     if(0)                                                                     // UART Echo
     while(UartRxBuffer_rd!=UartRxBuffer_wr){
         UART1_SendData8((uint8_t)UartRxBuffer[UartRxBuffer_rd]);
         while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
         if (++UartRxBuffer_rd>=UART_BUF_SIZE) UartRxBuffer_rd = 0;
     }
#endif


     if (1){   //tx data

       if (delay_to_wait_answer == 0){                                         //The timer between the packets being sent. Otherwise, heap data was not accepted.

         unsigned char DataToSend = 0;
         //if(UartRxBuffer_rd!=UartRxBuffer_wr){
         if(BufferTxRf_len>0){
           //unsigned char int_status[9];
           //SI446X_INT_STATUS(int_status);
           //if (((int_status[5]&0x03)!=0x00) || ((int_status[6]&0x03)!=0x00)) continue;
           //if ((int_status[6]&0x03)!=0x00) continue;


//           while((DataToSend<60) && (UartRxBuffer_rd!=UartRxBuffer_wr)) {wbuff[2+DataToSend++] = UartRxBuffer[UartRxBuffer_rd]; if (++UartRxBuffer_rd>=UART_BUF_SIZE) UartRxBuffer_rd = 0;}
while((DataToSend<60) && (DataToSend<BufferTxRf_len)) {wbuff[2+DataToSend] = BufferTxRf[DataToSend];DataToSend++;}
           wbuff[1] = (DataToSend)&0x3F;
           wbuff[0] = PACKET_DATA;


           delay_measure_tx = 0;

           //GPIO_WriteHigh(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);
           RFwrite(wbuff, PACKET_LEN);                                         // TX Data
           //GPIO_WriteLow(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);

           //Printf("Data sended %d\n\r", UartRxBuffer_rd/60);

           delay_measure_tx++;
           if (SYNCHRO_MODE) delay_to_wait_answer = delay_measure_tx*4;  //50; //Set timer between send the packets
           else delay_to_wait_answer = 10;  //50;

//#include <stdlib.h>
           delay_synchro = 500;//400 + rand()%200;
           BufferTxRf_len=0;
         }
#if SYNCHRO_MODE


         if ((DataToSend == 0) && ((NeedAnswer>0) || (delay_synchro>0))){      //Maintain communication by sync pulses.

           //Delay_ms(delay_measure_tx_middle);                                  //So as not to foul the broadcast.

           if (NeedAnswer) {wbuff[0] = NeedAnswer; NeedAnswer=0;}              // Pulse is Answer
           else if (delay_synchro>0) wbuff[0] = PACKET_ASK;                    // Pulse is Ask
           else wbuff[0] = PACKET_NONE;                                        // Pulse is None
           //wbuff[0] &=0x03;
           delay_measure_tx = 0;
           //GPIO_WriteHigh(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);
           RFwrite(wbuff, PACKET_LEN);                                         // TX Synchro Pulse
           //GPIO_WriteLow(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);

           delay_measure_tx++;
           delay_measure_tx_middle = delay_measure_tx;
           delay_to_wait_answer = delay_measure_tx*4;  //50;
         }
#endif

       }
     }
       //Delay_ms(1000);

     modem_busy=0;
     if (MODEM_PINCONTROL)HAL_GPIO_WritePin(PIN_SET_GPIO_Port, PIN_SET_Pin, GPIO_PIN_RESET);
}











/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
