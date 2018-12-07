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
  * COPYRIGHT(c) 2018 STMicroelectronics
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

#include "MPU9250.h"

#include "Si4463.h"
#include "radio_config_Si4463.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

extern pos_struct Est_A, Est_G, Est_M;
//pos_struct Est_G;
//pos_struct Est_M;
extern signed short MPU_temp;
extern signed int BMP_temp;
extern signed int BMP_press;

extern uint8_t aTxBuffer[1];

unsigned char PacketToSend[64];


unsigned char SETmainMotor = 0;	// 0 - off else - on
unsigned char SETmainMotorValue = 0;	//0-255
unsigned char SETservo1 = 0;		// 0 - off else - on
unsigned char SETservo1Value = 0;	//0-255
unsigned char SETservo2 = 0;		// 0 - off else - on
unsigned char SETservo2Value = 0;	//0-255
unsigned char SETindication = 0;	//0-indication off else - on

int ADC_data_0 = 0;
int ADC_data_1 = 0;

void CopyDataToTx(void);
void ReadCommandStream(unsigned char data);

#define READCMDSTREAM_ARRAY 10

unsigned char ReadCommandStreamArray[READCMDSTREAM_ARRAY][100];
unsigned char ReadCommandStreamArrayCnt[READCMDSTREAM_ARRAY];
int ReadCommandStreamArray_rd=0;
int ReadCommandStreamArray_wr=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void ModemControl(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



int modem_busy = 1;

#define PACKET_LEN 64

#define SYNCHRO_MODE 1

#define UART_BUF_SIZE 255

#define NONE_P  while(0){};	//GPIO_WriteHigh(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5); Delay_ms(1); GPIO_WriteLow(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5)
//#define NONE_P  Delay_ms(1)

typedef enum  {
	PACKET_NONE 				= 0,
	PACKET_DATA				= 1,
	PACKET_ASK				= 2,
	PACKET_ANSWER				= 3,
} PACKET_TYPE;


//volatile int time_delay=0;
//volatile int time_last_tx=0;
volatile int delay_to_wait_answer=0;
volatile int delay_measure_tx=0;
volatile int delay_synchro=0;
char NeedAnswer=0;
int delay_measure_tx_middle=0;

char UartRxBuffer[UART_BUF_SIZE];
volatile int UartRxBuffer_rd = 0;
volatile int UartRxBuffer_wr = 0;


unsigned char rbuff[PACKET_LEN],  wbuff[PACKET_LEN];



int timer1 = 0;
int SendDataUART_fl = 0;

int modem_timer = 5000;

void ReadModem(void);

struct timerForUpdate {			// This is time settings for some processes
	int counter;		//startup time
	int time_limit;		//polling interval
} timer_MPU_temp = {5000, 500},
		timer_MPU_acc = {5000, 500},
		timer_MPU_gyro = {5000, 500},
		timer_MPU_mag = {5000, 500},
		timer_BMP_temp = {6000, 2000},
		timer_BMP_press = {5000, 2000},
		timer_ADC = {5000, 1000},
		timer_ModemControl = {5000, 100},
		timer_PrepareDataToTx = {5000, 1000},
		timer_ReadStream = {5000, 50};



void HAL_SYSTICK_Callback(void)
{
	if (--modem_timer<=0){ReadModem();modem_timer=10;}

	if(delay_to_wait_answer>0) delay_to_wait_answer--;
	if(delay_synchro>0) delay_synchro--;
	delay_measure_tx++;

// Updater time for counters

	if (timer_MPU_temp.counter>0) timer_MPU_temp.counter--;
	if (timer_MPU_acc.counter>0) timer_MPU_acc.counter--;
	if (timer_MPU_gyro.counter>0) timer_MPU_gyro.counter--;
	if (timer_MPU_mag.counter>0) timer_MPU_mag.counter--;
	if (timer_BMP_temp.counter>0) timer_BMP_temp.counter--;
	if (timer_BMP_press.counter>0) timer_BMP_press.counter--;
	if (timer_ADC.counter>0) timer_ADC.counter--;
	if (timer_ModemControl.counter>0) timer_ModemControl.counter--;
	if (timer_PrepareDataToTx.counter>0) timer_PrepareDataToTx.counter--;
	if (timer_ReadStream.counter>0) timer_ReadStream.counter--;

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
		CopyDataToTx();
		//HAL_UART_Transmit(&huart1, PacketToSend, sizeof(CMD2PacketStruct)-2, 500);	//41

		int r=0;
		for (r=0; r<(sizeof(CMD2PacketStruct)-2); r++){
			UartRxBuffer[UartRxBuffer_wr] = PacketToSend[r];
			if (++UartRxBuffer_wr>=UART_BUF_SIZE) UartRxBuffer_wr = 0;
		}

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



void Main_Callback(void)
{
	ModemControl();

	if (timer_ADC.counter==0)  {		ADC_data_0 = Read_ADC_Channel(0);
										ADC_data_0=(ADC_data_0*3300)/0xFFF;	timer_ADC.counter = timer_ADC.time_limit;}
	if (timer_ModemControl.counter==0)  {ModemControl();	timer_ModemControl.counter = timer_ModemControl.time_limit;}
	if (timer_PrepareDataToTx.counter==0){PrepareDataToTx();timer_PrepareDataToTx.counter = timer_PrepareDataToTx.time_limit;}
	if (timer_ReadStream.counter==0)  {	ReadStream();		timer_ReadStream.counter = timer_ReadStream.time_limit;}
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
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim3);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  TIM3->CCR3 = 1500;	//main
  TIM3->CCR2 = 1500;	//servo1
  TIM3->CCR1 = 1000;	//servo2


  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);	// Enable Interrupt

  ///TEST

  //I2C_WriteBuffer(hi2c1, 0x68, 1);
  //I2C_ReadBuffer(hi2c1, 0x68, 1);

  int temp = MPU_Init();						// Init Chip MPU9250
  if (temp == 0) Printf("Device init\n\r");
  else if (temp == -1) Printf("I2C error, id is %d\n\r", aTxBuffer[0]);
  else if (temp == -2) Printf("AXIS error, id is %d\n\r", aTxBuffer[0]);
  else if (temp == -3) Printf("MAG error, id is %d\n\r", aTxBuffer[0]);
  else if (temp == -4) Printf("BMP error, id is %d\n\r", aTxBuffer[0]);
  else Printf("Error nothing\n\r");

	BMPInit();		// Init Chip BMP180


  HAL_GPIO_WritePin(SI_SDN_GPIO_Port, SI_SDN_Pin, GPIO_PIN_SET);	//Shut down on
  HAL_Delay(1000);
  HAL_GPIO_WritePin(SI_SDN_GPIO_Port, SI_SDN_Pin, GPIO_PIN_RESET);	//Shut down off
  HAL_Delay(1000);

  unsigned char buffer[16];
  do{

  	SI446X_PART_INFO(buffer);

  	Printf("Si data is : 0x%x, 0x%x\n\r", buffer[2], buffer[3]);
  	if ((buffer[2] == 0x44) && (buffer[3]== 0x63)) Printf("Dev is si4463\n\r");
  	HAL_Delay(1000);

  }while((buffer[3]!= 0x63));

  RFinit();			//Set Settings Config to Si4463

  SI446X_FIFOINFO(0, 0, 1, 1);		// BUFFER CLEAR

  changeState(STATE_RX);			// Set state to RX


Printf("Started...\n\r");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



		if (timer_MPU_temp.counter==0) {	Get_temp();			timer_MPU_temp.counter = timer_MPU_temp.time_limit;}
		if (timer_MPU_acc.counter==0)  {	Get_acc();			timer_MPU_acc.counter = timer_MPU_acc.time_limit;}
		if (timer_MPU_gyro.counter==0)  {	Get_gyro();			timer_MPU_gyro.counter = timer_MPU_gyro.time_limit;}
		if (timer_MPU_mag.counter==0)  {	Get_mag();			timer_MPU_mag.counter = timer_MPU_mag.time_limit;}
		if (timer_BMP_temp.counter==0)  {	GetBmpTemp();		timer_BMP_temp.counter = timer_BMP_temp.time_limit;}
		if (timer_BMP_press.counter==0)  {	GetBmpPress();		timer_BMP_press.counter = timer_BMP_press.time_limit;}
		if (timer_ADC.counter==0)  {		ADC_data_0 = Read_ADC_Channel(0);
											ADC_data_0=(ADC_data_0*3300)/0xFFF;	timer_ADC.counter = timer_ADC.time_limit;}
		if (timer_ModemControl.counter==0)  {ModemControl();	timer_ModemControl.counter = timer_ModemControl.time_limit;}
		if (timer_PrepareDataToTx.counter==0){PrepareDataToTx();timer_PrepareDataToTx.counter = timer_PrepareDataToTx.time_limit;}
		if (timer_ReadStream.counter==0)  {	ReadStream();		timer_ReadStream.counter = timer_ReadStream.time_limit;}



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
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
  htim3.Init.Prescaler = 64;
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
  huart1.Init.BaudRate = 115200;
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
  HAL_GPIO_WritePin(SI_SDN_GPIO_Port, SI_SDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PIN_SET_Pin SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = PIN_SET_Pin|SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
void CopyDataToTx(void)
{

	  CMD2PacketStruct * ptr = (CMD2PacketStruct *)&PacketToSend[0];

	  ptr->startByte = '$';
	  ptr->cmd = 0x2;
	  ptr->len = sizeof(CMD2PacketStruct)-5-2;
	  ptr->voltage = ADC_data_0&0xFFFF;
	  ptr->current = ADC_data_1&0xFFFF;

	  ptr->mainMotor = SETmainMotor;
	  ptr->mainMotorValue = SETmainMotorValue;
	  ptr->servo1 = SETservo1;
	  ptr->servo1Value = SETservo1Value;
	  ptr->servo2 = SETservo2;
	  ptr->servo2Value = SETservo2Value;
	  ptr->indication = SETindication;

	  ptr->dofAx = Est_A.x;
	  ptr->dofAy = Est_A.y;
	  ptr->dofAz = Est_A.z;

	  ptr->dofGx = Est_G.x;
	  ptr->dofGy = Est_G.y;
	  ptr->dofGz = Est_G.z;

	  ptr->dofMx = Est_M.x;
	  ptr->dofMy = Est_M.y;
	  ptr->dofMz = Est_M.z;

	  ptr->temp1 = MPU_temp;
	  ptr->temp2 = BMP_temp;

	  ptr->press = BMP_press;
	  ptr->crc = 0x00;
	  ptr->endByte = '#';

	  unsigned char * pptr = &PacketToSend[3];
	  int i=0;
	  unsigned char matchCRC = 0;
	  for (i=0;i<(sizeof(CMD2PacketStruct)-5-2); i++)matchCRC ^= *pptr++;

	  ptr->crc = matchCRC;

}



int position = 0;
int data_len = 0;

unsigned char PacketRx[255];
int PacketRxB = 0;
unsigned char PacketCRC = 0;


void ReadCommandStream(unsigned char data)
{


	unsigned char cTemp = data;//(uint8_t)(huart1.Instance->DR & (uint8_t)0x00FF);
	//HAL_UART_Transmit(&huart1, &UART_rx_bufer[UART_rx0_index], 1, 100);
	//HAL_UART_Receive

	PacketCRC ^= cTemp;

	if ((position == 0) && (cTemp == '$')) {position++; PacketCRC = 0;}
	else if ((position == 1) && (cTemp == 0x12)) position++;
	else if (position == 2) {data_len = cTemp; position++; PacketRxB = 0; PacketCRC = 0;}
	else if ((position == 3) && (data_len > 0)) {PacketRx[PacketRxB++] = cTemp; data_len--;}
	else if ((position == 3) && (data_len <= 0)) {position++;} //Check CRC

	else if ((position == 4) && (cTemp == '#')){

		Printf("Motorv= %d\n\r", PacketRx[1]);

		SETmainMotor = PacketRx[0];
		SETmainMotorValue = PacketRx[1];
		SETservo1 = PacketRx[2];
		SETservo1Value = PacketRx[3];
		SETservo2 = PacketRx[4];
		SETservo2Value = PacketRx[5];
		SETindication = PacketRx[6];

		if (SETmainMotor != 0) TIM3->CCR1 = 1000 + ((SETmainMotorValue*1000)/0xFF);
		else TIM3->CCR1 = 1000;

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
		else TIM3->CCR2 = 0;

		if (SETservo2 != 0) {
			int setValue;
			signed char scTemp = *(signed char *)&SETservo2Value;
			//if (SETservo2Value<=0x80) setValue = 1500 + ((SETservo2Value*500)/0x80);
			//else { setValue = 1500 - ((SETservo2Value*500)/0x80);}	//SETservo2Value = (~SETservo2Value)+1;
			setValue = 1500 + ((scTemp*500)/0x7F);
			if ((setValue>2000) || (setValue<1000)) setValue = 1500;
			TIM3->CCR3 = setValue;
		}
		else TIM3->CCR3 = 0;



		position = 0;
	}
	else position = 0;


}

#define MODEM_PINCONTROL 1




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

         char DataToSend = 0;
         if(UartRxBuffer_rd!=UartRxBuffer_wr){
           //unsigned char int_status[9];
           //SI446X_INT_STATUS(int_status);
           //if (((int_status[5]&0x03)!=0x00) || ((int_status[6]&0x03)!=0x00)) continue;
           //if ((int_status[6]&0x03)!=0x00) continue;


           while((DataToSend<60) && (UartRxBuffer_rd!=UartRxBuffer_wr)) {wbuff[2+DataToSend++] = UartRxBuffer[UartRxBuffer_rd]; if (++UartRxBuffer_rd>=UART_BUF_SIZE) UartRxBuffer_rd = 0;}
           wbuff[1] = (DataToSend)&0x3F;
           wbuff[0] = PACKET_DATA;


           delay_measure_tx = 0;

           //GPIO_WriteHigh(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);
           RFwrite(wbuff, PACKET_LEN);                                         // TX Data
           //GPIO_WriteLow(GPIOB, (GPIO_Pin_TypeDef)GPIO_PIN_5);

           //Printf("Data sended %d\n\r", UartRxBuffer_rd/60);

           if (SYNCHRO_MODE) delay_to_wait_answer = delay_measure_tx*5;  //50; //Set timer between send the packets
           else delay_to_wait_answer = 10;  //50;
           delay_synchro = 500;
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

           delay_measure_tx_middle = delay_measure_tx;
           delay_to_wait_answer = delay_measure_tx*5;  //50;
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
