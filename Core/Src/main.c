/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Uncomment the desired mode for stepper motor operation */
//#define MODE_WAVE_DRIVE
#define MODE_FULL_STEP
//#define MODE_HALF_STEP


#ifdef MODE_WAVE_DRIVE
#define MODE  ((uint16_t[]){0x1, 0x10,  0x100, 0x1000}) //Clockwise
#define MODE_  ((uint16_t[]){0x1000,  0x100, 0x10, 0x1}) //AntiClockwise
#define STEPS_NUMBER 4
#endif
#ifdef MODE_FULL_STEP
#define MODE ((uint16_t[]){0x11, 0x110,  0x1100, 0x1001}) //Clockwise
#define MODE_ ((uint16_t[]){0x1001,  0x1100, 0x110, 0x11}) //AntiClockwise
#define STEPS_NUMBER 4
#endif
#ifdef MODE_HALF_STEP
#define MODE ((uint16_t[]){0x1, 0x11, 0x10, 0x110, 0x100, 0x1100, 0x1000, 0x1001}) //Clockwise
#define MODE_ ((uint16_t[]){0x1001, 0x1000, 0x1100, 0x100, 0x110, 0x10, 0x11, 0x1}) //AntiClockwise
#define STEPS_NUMBER 8
#endif

/* USER CODE END PD */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef struct {
	uint8_t delay_flag_EXTI15_10 : 2;
	uint32_t delay_exti;
	uint32_t rtc_ck_catch;
} RTC_data;

typedef struct {
    uint16_t clock_wise_sequence[STEPS_NUMBER];
    uint16_t anti_clockwise_sequence[STEPS_NUMBER];
    uint16_t speed;
    uint16_t torque;
    uint16_t temperature;
    struct {
        uint8_t direction : 1;
        uint8_t stop_flag :1;
        uint8_t control_method :1;
        uint8_t steps_counter : 3;
    } Flags;
}StepMotorData;

/* USER CODE END PTD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
RTC_data rtc_struct;
StepMotorData motor1;

uint32_t bluetooth_rx;
#define BUFFER_LENGTH 10
uint32_t buffer_bluetooth_rx[BUFFER_LENGTH];
int32_t buffer_pos;

uint16_t bluetooth_tx;
uint8_t buffer_bluetooth_tx[BUFFER_LENGTH];
int16_t buffer_pos_tx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void RTC_config(void);
void TIM3_config(void);
void ADC1_config(void);
void DMA1_Channel1_config(void);
void USART1_config(void);
void DMA1_Channel4_config(void);
void DMA1_Channel5_config(void);

void EXTI_config(void);
void NVIC_config(void);
void GPIO_config(void);
void EXTI15_10_IRQHandler(void);
void RTC_IRQHandler(void);
void TIM3_IRQHandler(void);
void USART1_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/* Structure initialization */
	rtc_struct.delay_flag_EXTI15_10 =0;
	rtc_struct.delay_exti =0;
	rtc_struct.rtc_ck_catch =0;

	memcpy(motor1.clock_wise_sequence, MODE, sizeof(MODE));
	memcpy(motor1.anti_clockwise_sequence, MODE_, sizeof(MODE_));
	motor1.Flags.direction = 0;
	motor1.Flags.steps_counter = 0;
	motor1.Flags.stop_flag = 0;
	motor1.Flags.control_method =0;

	bluetooth_rx = 0;
	memset(buffer_bluetooth_rx, 0, sizeof(buffer_bluetooth_rx));
	buffer_pos = 0;

	bluetooth_tx =0;
	memset(buffer_bluetooth_tx, 0, sizeof(buffer_bluetooth_tx));
	buffer_pos_tx = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();

  /* USER CODE BEGIN Init */
	SysTick->CTRL &= ~(1);
	SysTick->LOAD =8000; //8MHz /8000 = 1000Hz == 1/1ms
	SysTick->CTRL |= 1; // Enable SysTick
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  GPIO_config();
  /* USER CODE BEGIN 2 */
  RTC_config();
  TIM3_config();
  ADC1_config();
  DMA1_Channel1_config();
  USART1_config();
  DMA1_Channel4_config();
  DMA1_Channel5_config();

  EXTI_config();
  NVIC_config();

  //  Try BackUp registers  (For BackUp registers u need RCC->APB1ENR |= (1<<27); (u have this in RCC config)// Bit 27 BKPEN: Backup interface clock enable)
  //PWR->CR |= (1<<8); // 1: Access to RTC and Backup registers enabled
  //BKP->DR1 = 0xAAA; // Write data to backup register 1
  PWR->CR &= ~(1<<8); //Disable write access to the backup registers by clearing the DBP bit in the PWR_CR register to prevent accidental writes.
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;


  /* GPIO Ports Clock Enable */
  RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Clock enable
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;    // Enable clock for AFIO for EXTI
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //Clock enable
  RCC->AHBENR |= RCC_AHBENR_DMA1EN; //Clock enable
  //RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //Clock enable
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  /*Configuration clock for RTC */
  RCC->APB1ENR |= (1<<28); // Bit 28 PWREN: Power interface clock enable
  RCC->APB1ENR |= (1<<27); // Bit 27 BKPEN: Backup interface clock enable
  PWR->CR |= (1<<8); // 1: Access to RTC and Backup registers enabled
  RCC->CSR |= 1; // LSION:   LSI ON
  while((RCC->CSR & (1<<1)) != (1<<1)); // Bit 1 LSIRDY: Internal low-speed oscillator ready
  //RCC->BDCR |= 1; // Bit 0 LSEON: External low-speed oscillator enable
  //while((RCC->BDCR & (1<<1)) != (1<<1));  !!!!!!!!! УЖЕ НА ЭТОМ МОМЕНТЕ СТОПИТЬСЯ. LSE НЕ МОЖЕТ ПРИГОТОВИТЬСЯ И Я НЕ ЗНАЮ ЧЕГО. LSI наоборот работает
  RCC->BDCR |= (1<<9); //Bits 9:8 RTCSEL[1:0]: RTC clock source selection 10: LSI oscillator clock used as RTC clock
  //RCC->BDCR |= (1<<8); // Bits 9:8 RTCSEL[1:0]: RTC clock source selection 01: LSE oscillator clock used as RTC clock
  //RCC->BDCR |= (1<<2); //Bit 2 LSEBYP: External low-speed oscillator bypass
  RCC->BDCR |= (1<<15); //Bit 15 RTCEN: RTC clock enable
  PWR->CR &= ~(1<<8); //Disable write access to the backup registers by clearing the DBP bit in the PWR_CR register to prevent accidental writes.


  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void GPIO_config(void)
{
  /* Configuration pins for ADC1 : IN2 and IN3 and IN4 */
  GPIOA->CRL &= ~(1<<12); // Configuration Analog mode for PA3 = IN3
  GPIOA->CRL &= ~(1<<8); // Configuration Analog mode for PA2 = IN2
  GPIOA->CRL &= ~(1<<16); // Configuration Analog mode for PA4 = IN4

  /* Configuration pins PB12 and PB13 and PB14 as Input with pull-down for buttons*/
  GPIOB->CRH &= ~((1<<18)|(1<<22)|(1<<26));
  GPIOB->CRH |= ((1<<19)|(1<<23)|(1<<27));
  GPIOB->ODR &= ~((1<<12)|(1<<13)|(1<<14)); // Enable pull-down resistor

  // Enable GPIOC peripheral clock
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

  // Configure GPIOC pin 13 as output push-pull
  GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
  GPIOC->CRH |= GPIO_CRH_MODE13_0;

	// Configure PA6 as Alternate function push-pull
	GPIOA->CRL |= (1<<25);            //MODE6 set in 10: Output mode, max speed 2 MHz.
	GPIOA->CRL &= ~(1<<26); //Because of Reset state on CNF6 (01) we needed set 26 bit (CNF6[0]) to 0
	GPIOA->CRL |= (1<<27);   // CNF6[1], CNF6 set in 10: Alternate function output Push-pull
	// Configure PA7 as Alternate function push-pull
	GPIOA->CRL |= (1<<29);            //MODE7 set in 10: Output mode, max speed 2 MHz.
	GPIOA->CRL &= ~(1<<30); //Because of Reset state on CNF7 (01) we needed set 26 bit (CNF7[0]) to 0
	GPIOA->CRL |= (1<<31);   // CNF7[1], CNF7 set in 10: Alternate function output Push-pull
	// Configure PB0 as Alternate function push-pull
	GPIOB->CRL |= (1<<1);            //MODE0 set in 10: Output mode, max speed 2 MHz.
	GPIOB->CRL &= ~(1<<2); //Because of Reset state on CNF0 (01) we needed set 26 bit (CNF0[0]) to 0
	GPIOB->CRL |= (1<<3);   // CNF0[1], CNF0 set in 10: Alternate function output Push-pull
	// Configure PB1 as Alternate function push-pull
	GPIOB->CRL |= (1<<5);            //MODE1 set in 10: Output mode, max speed 2 MHz.
	GPIOB->CRL &= ~(1<<6); //Because of Reset state on CNF1 (01) we needed set 26 bit (CNF1[0]) to 0
	GPIOB->CRL |= (1<<7);   // CNF1[1], CNF1 set in 10: Alternate function output Push-pull

	// Configure PA10 as RX pin for USART1 Input Pull-UP
	GPIOA->CRH &= ~(1<<10); // Input PULL-UP
	GPIOA->CRH |= (1<<11);
	// Configure PA9 as TX pin for USART1 Alternate function output Push-pull
	GPIOA->CRH &= ~(1<<6);
	GPIOA->CRH |= (1<<7);
	GPIOA->CRH |= (1<<5);
}

/* USER CODE BEGIN 4 */

void RTC_config(void)
{
	PWR->CR |= (1<<8); // 1: Access to RTC and Backup registers enabled
	while((RTC->CRL &(1<<5)) != (1<<5)); // Poll RTOFF, wait until its value goes to 1.
	RTC->CRL |= (1<<4); //Bit 4 CNF: Configuration flag 1: Enter configuration mode.
	// Configuration
	RTC->CRH |=1;// Bit 0 SECIE: Second interrupt enable
	RTC->PRLL &= ~0xFFFF; //Reset to 0
	RTC->PRLL |= 399; // f_TR_CLK = 40000/(399+1)  ~~ 100Hz
	RTC->CRL &= ~(1<<4); //0: Exit configuration mode (start update of RTC registers).
	while((RTC->CRL &(1<<5)) == (0<<5)); // Poll RTOFF, wait until its value goes to 1
	PWR->CR &= ~(1<<8); //Disable write access to the backup registers by clearing the DBP bit in the PWR_CR register to prevent accidental writes.
}


void ADC1_config(void)
{
	ADC1->CR1 |= (1<<8); // Bit 8 SCAN: Scan mode 1: Scan mode enabled
	ADC1->CR2 |= (1<<20); //Bit 20 EXTTRIG: External trigger conversion mode for regular channels
	ADC1->CR2 |= (1<<19); //Bits 19:17 EXTSEL[2:0]: External event select for regular group 100: Timer 3 TRGO event
	ADC1->CR2 |= (1<<8); //Bit 8 DMA: Direct memory access mode 1: DMA mode enabled
	ADC1->CR2 |= 1; //ADC1 wake up
	ADC1->SQR1 |= (1<<20); // Bits 23:20 L[3:0]: Regular channel sequence length 0010: 2 conversions
	ADC1->SQR3 |= 2;
	ADC1->SQR3 |= (3<<5);
	ADC1->SQR3 |= (4<<10); //ADC regular sequence
	uint32_t i;
	i=0;
	while( i < 100000) i++; //delay between power up and start of conversion in ADC1
	ADC1->CR2 |= 1; //ADC1 Start of Conversion

}

void DMA1_Channel1_config(void)
{
	DMA1_Channel1->CCR |= (1<<12); //Bits 13:12 PL[1:0]: Channel priority level 01: Medium
	DMA1_Channel1->CCR |= (1<<10); // Bits 11:10 MSIZE[1:0]: Memory size 01: 16-bits
	DMA1_Channel1->CCR |= (1<<8); //Bits 9:8 PSIZE[1:0]: Peripheral size 01: 16-bits
	DMA1_Channel1->CCR |= (1<<7); //Bit 7 MINC: Memory increment mode 1: Memory increment mode enabled
	DMA1_Channel1->CCR |= (1<<5); //Bit 5 CIRC: Circular mode 1: Circular mode enabled
	DMA1_Channel1->CNDTR =3; //Number of data to transfer
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
	DMA1_Channel1->CMAR = (uint32_t)&(motor1.speed);
	DMA1_Channel1->CCR |= 1; // DMA channel 1 enable
}

void TIM3_config(void)
{
	TIM3->CR1 |= (1<<7); // Bit 7 ARPE: Auto-reload preload enable
	TIM3->CR1 |= (1<<4); // Bit 4 DIR: Direction  1: Counter used as downcounter
	TIM3->CR2 |= (1<<5); // Master mode selection: 010: Update - The update event is selected as trigger output (TRGO).
	//TIM3->DIER |= (1<<8); // Bit 8 UDE: Update DMA request enable
	TIM3->DIER |= 1; // Bit 0 UIE: Update interrupt enable
	TIM3->CCMR1 |= ((1<<14)|(1<<13));
	TIM3->CCMR1 |= (1<<11);
	TIM3->CCMR1 &=~((1<<9)|(1<<8));
	TIM3->CCMR1 |= ((1<<6)|(1<<5));// Bits 6:4 OC1M: Output compare 1 mode 110: PWM mode 1
	TIM3->CCMR1 |= (1<<3); //Bit 3 OC1PE: Output compare 1 preload enable
	TIM3->CCMR1 &=~((1<<1)|1); // 00: CC1 channel is configured as output.
	TIM3->CCMR2 |= ((1<<14)|(1<<13));
	TIM3->CCMR2 |= (1<<11);
	TIM3->CCMR2 &=~((1<<9)|(1<<8));
	TIM3->CCMR2 |= ((1<<6)|(1<<5));
	TIM3->CCMR2 |= (1<<3);
	TIM3->CCMR2 &=~((1<<1)|1);
	TIM3->PSC  |= 15;  // freq_CK_CNT = 8MHz/(PSC +1) = 500 000 Hz
	TIM3->ARR &= ~0xFFFF;             // Because of Reset value of ARR
	TIM3->ARR |= 1000; // freq_UpdateEvent =freq_CK_CNT/ARR = 500Hz


	TIM3->EGR |=1; // Re-initialize the counter and generates an update of the registers
	TIM3 ->CR1 |= 0x1;       // enable Counter
}


void USART1_config(void)
{
	/* Configuration USART1 */
	USART1->BRR |=  0x341;//(0x3415); //Baud rate register //9600
	USART1->CR1 |= (1<<3); //	Bit 3 TE: Transmitter enable
	USART1->CR1 |= (1<<2); // ,  Bit 2 RE: Receiver enable
	USART1->CR1 |= (1<<7); // Bit 7 TXEIE: TXE interrupt enable
//	USART1->CR1 |= (1<<5); // Bit 5 RXNEIE: RXNE interrupt enable
	USART1->CR3 |= (1<<7); // Bit 7 DMAT: DMA enable transmitter
	USART1->CR3 |= (1<<6); // Bit 6 DMAR: DMA enable receiver
	//USART1->CR1 |= (1<<13); // Bit 13 UE: USART enable
}
void DMA1_Channel4_config(void)
{
	DMA1_Channel4->CCR |= (1<<12); //Bits 13:12 PL[1:0]: Channel priority level
	DMA1_Channel4->CCR &= ~((1<<10)|(1<<11)); // Bits 11:10 MSIZE[1:0]: Memory size 00: 8-bits
	DMA1_Channel4->CCR |= (1<<8); //Bits 9:8 PSIZE[1:0]: Peripheral size 01: 16-bits
	DMA1_Channel4->CCR |= (1<<7); //Bit 7 MINC: Memory increment mode 1: Memory increment mode enabled
	DMA1_Channel4->CCR |= (1<<5); //Bit 5 CIRC: Circular mode 1: Circular mode enabled
	DMA1_Channel4->CCR |= (1<<4); //Bit 4 DIR: Data transfer direction 1: Read from memory
	DMA1_Channel4->CCR |= (1<<1); // Bit 1 TCIE: Transfer complete interrupt enable
	DMA1_Channel4->CNDTR |=BUFFER_LENGTH;
	DMA1_Channel4->CPAR = &(USART1->DR);
	DMA1_Channel4->CMAR = &(buffer_bluetooth_tx);

	//DMA1_Channel4->CCR |= 1; //Channel enable
}
void DMA1_Channel5_config(void)
{
	DMA1_Channel5->CCR |= (1<<12); //Bits 13:12 PL[1:0]: Channel priority level
	DMA1_Channel5->CCR |= (1<<10); // Bits 11:10 MSIZE[1:0]: Memory size 01: 16-bits
	DMA1_Channel5->CCR |= (1<<8); //Bits 9:8 PSIZE[1:0]: Peripheral size 01: 16-bits
	//DMA1_Channel5->CCR |= (1<<7); //Bit 7 MINC: Memory increment mode 1: Memory increment mode enabled
	DMA1_Channel5->CCR |= (1<<5); //Bit 5 CIRC: Circular mode 1: Circular mode enabled
	DMA1_Channel5->CCR |= (1<<1); // Bit 1 TCIE: Transfer complete interrupt enable
	DMA1_Channel5->CNDTR = 1; //Bit 1 TCIE: Transfer complete interrupt enable
	DMA1_Channel5->CPAR = &(USART1->DR);
	DMA1_Channel5->CMAR = &bluetooth_rx;

	DMA1_Channel5->CCR |= 1; //Channel enable
}



void EXTI_config(void)
{

	EXTI->IMR |= (1<<12); //Interrupt request from Line 12 is not masked
	EXTI->RTSR |=(1<<12); //Rising trigger event configuration bit of line 12
	AFIO->EXTICR[3] |= 1; // select the source input for EXTI12 external interrupt.
	EXTI->IMR |= (1<<13); //Interrupt request from Line 13 is not masked
	EXTI->RTSR |=(1<<13); //Rising trigger event configuration bit of line 13
	AFIO->EXTICR[3] |= (1<<4); // select the source input for EXTI13 external interrupt.
	EXTI->IMR |= (1<<14); //Interrupt request from Line 14 is not masked
	EXTI->RTSR |=(1<<14); //Rising trigger event configuration bit of line 14
	AFIO->EXTICR[3] |= (1<<8); // select the source input for EXTI14 external interrupt.
}

void NVIC_config(void)
{
	// Configuration NVIC for EXTI15_10
	NVIC -> IP[40] |= 0x10;        //Priority
	NVIC ->ISER[1] |= (1<<8);      // Enable global interrupt on EXTI15_10
	// Configuration NVIC for RTC
	NVIC -> IP[3] |= 0x20;//0x0;        //Priority
	NVIC ->ISER[0] |= (1<<3);      // Enable global interrupt on RTC
	// Configuration NVIC for TIM3
	NVIC -> IP[29] |= 0x20; //priority
	NVIC ->ISER[0] |= (1<<29); // Enable global interrupt on TIM3
	// Configuration NVIC for USART1
	NVIC -> IP[37] |= 0x30; //priority
	NVIC ->ISER[1] |= (1<<5); // Enable global interrupt on USART1
	//Configuration NVIC for DMA1_Channel5
	NVIC -> IP[15] |= 0x10; //priority
	NVIC ->ISER[0] |= (1<<15); // Enable global interrupt on DMA1_Channel5
	//Configuration NVIC for DMA1_Channel4
	NVIC -> IP[14] |= 0x10; //priority
	NVIC ->ISER[0] |= (1<<14); // Enable global interrupt on DMA1_Channel4
}


void EXTI15_10_IRQHandler(void)
{
	GPIOC ->ODR ^=(1<<13);

	if(rtc_struct.delay_flag_EXTI15_10 == 0)
	{
		NVIC ->ICER[1] |= (1<<8);      // DisEnable global interrupt on EXTI15_10
		rtc_struct.delay_exti = 100;
		rtc_struct.delay_flag_EXTI15_10 =1;
		return 0;
	}


	if((EXTI->PR & (1<<12)) == (1<<12)) //trigger request occurred on line EXTI12
	{
		EXTI->PR |= (1<<12); // Clearing Pending bit for line EXTI12
		motor1.Flags.direction ^=1; // toggle direction of motor1 rotation

		PWR->CR |= (1<<8); // 1: Access to RTC and Backup registers enabled
		BKP->DR1 = RTC->CNTL; // Write data to backup register 1
		PWR->CR &= ~(1<<8); //Disable write access to the backup registers by clearing the DBP bit in the PWR_CR register to prevent accidental writes.
	}
	if((EXTI->PR & (1<<13)) == (1<<13)) //trigger request occurred on line EXTI13
	{
		EXTI->PR |= (1<<13); // Clearing Pending bit for line EXTI13
		motor1.Flags.stop_flag ^=1; // toggle ON/OFF motor1
		if(BKP->DR1 == 0) NVIC ->ICER[0] |= (1<<3);
	}
	if((EXTI->PR & (1<<14)) == (1<<14)) //trigger request occurred on line EXTI14
	{
		EXTI->PR |= (1<<14); //// Clearing Pending bit for line EXTI14
		motor1.Flags.control_method ^=1;
		if(motor1.Flags.control_method == 0)
		{
			USART1->CR1 &= ~(1<<13); // Bit 13 UE: USART Disable
			ADC1->CR2 &=~1; // Disable ADC and go to power down mode
			DMA1_Channel1->CCR &= ~1; // DMA channel 1 disable

			ADC1->CR1 |= (1<<8); // Bit 8 SCAN: Scan mode 1: Scan mode enabled
			ADC1->CR2 |= (1<<20); //Bit 20 EXTTRIG: External trigger conversion mode for regular channels
			ADC1->CR2 |= (1<<19); //Bits 19:17 EXTSEL[2:0]: External event select for regular group 100: Timer 3 TRGO event
			ADC1->CR2 |= (1<<8); //Bit 8 DMA: Direct memory access mode 1: DMA mode enabled
			ADC1->SQR1 = 0;
			ADC1->SQR1 = (1<<20); // Bits 23:20 L[3:0]: Regular channel sequence length 0010: 3 conversions
			ADC1->SQR3 = 0;
			ADC1->SQR3 |= 2;
			ADC1->SQR3 |= (3<<5);
			//ADC1->SQR3 |= (4<<10); //ADC regular sequence
			ADC1->CR2 |= 1; //ADC1 wake up
			uint32_t i;
			i=0;
			while( i < 100000) i++; //delay between power up and start of conversion in ADC1
			ADC1->CR2 |= 1; //ADC1 Start of Conversion

			DMA1_Channel1->CCR |= (1<<7); //Bit 7 MINC: Memory increment mode 1: Memory increment mode enabled
			DMA1_Channel1->CNDTR =3; //Number of data to transfer
			DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
			DMA1_Channel1->CMAR = (uint32_t)&(motor1.speed);
			DMA1_Channel1->CCR |= 1; // DMA channel 1 enable
		}
		else
		{
			ADC1->CR2 &=~1; // Disable ADC and go to power down mode
			USART1->CR1 |= (1<<13); // Bit 13 UE: USART enable

			DMA1_Channel1->CCR &= ~1; // DMA channel 1 disable

			ADC1->SQR1 = 0; // Bits 23:20 L[3:0]: Regular channel sequence length 0000: 1 conversions
			ADC1->SQR3 = 0;
			ADC1->SQR3 |= 4; //ADC regular sequence
			ADC1->CR2 |= 1; //ADC1 wake up
			uint32_t i;
			i=0;
			while( i < 100000) i++; //delay between power up and start of conversion in ADC1
			ADC1->CR2 |= 1; //ADC1 Start of Conversion

			DMA1_Channel1->CCR &= ~(1<<7); //Bit 7 MINC: Memory increment mode
			DMA1_Channel1->CNDTR =1; //Number of data to transfer
			DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
			DMA1_Channel1->CMAR = (uint32_t)&(motor1.temperature);
			DMA1_Channel1->CCR |= 1; // DMA channel 1 enable
		}
	}

	rtc_struct.delay_flag_EXTI15_10 = 0;
}

void RTC_IRQHandler(void)
{
	PWR->CR |= (1<<8); // 1: Access to RTC and Backup registers enabled
	RTC->CRL &= ~(1); // Clear Second flag Bit 0 SECF: Second flag

	if(rtc_struct.delay_flag_EXTI15_10 ==1)
	{
		rtc_struct.delay_flag_EXTI15_10 = 2;
		rtc_struct.rtc_ck_catch = RTC->CNTL;
	}
	if ((rtc_struct.delay_flag_EXTI15_10 == 2) && (RTC->CNTL - rtc_struct.rtc_ck_catch >= rtc_struct.delay_exti))
	{
		rtc_struct.delay_flag_EXTI15_10 = 3;
		NVIC ->ISER[1] |= (1<<8);      // Enable global interrupt on EXTI15_10
	}

	PWR->CR &= ~(1<<8); //Disable write access to the backup registers by clearing the DBP bit in the PWR_CR register to prevent accidental writes.
}

void TIM3_IRQHandler(void)
{
	TIM3->SR &=~1; // Clear pending bit
	if(motor1.Flags.direction == 0 && motor1.Flags.stop_flag !=1)
	{
		TIM3->CCER = motor1.clock_wise_sequence[motor1.Flags.steps_counter];
		motor1.Flags.steps_counter < (STEPS_NUMBER-1) ? (motor1.Flags.steps_counter++) : (motor1.Flags.steps_counter = 0);

	}
	else if (motor1.Flags.direction ==1 && motor1.Flags.stop_flag !=1)
	{
		TIM3->CCER = motor1.anti_clockwise_sequence[motor1.Flags.steps_counter];
		motor1.Flags.steps_counter < (STEPS_NUMBER-1) ? (motor1.Flags.steps_counter++) : (motor1.Flags.steps_counter = 0);
	}
	else TIM3->CCER = 0;

	TIM3->ARR = ((motor1.speed *(2000-250))/(4095-0)) + 250; // Range of values 250-2000. In Freq 2000 Hz - 250 Hz
	TIM3->CCR1=TIM3->CCR2=TIM3->CCR3=TIM3->CCR4= ((motor1.torque *(TIM3->ARR))/(4095-0));


}

void DMA1_Channel5_IRQHandler(void)
{
	if ((DMA1->ISR &(1<<17)) == (1<<17))  //TCIF5: Channel 5 transfer complete flag
			{
				DMA1_Channel5->CCR &= ~1; //Bit 0 EN: Channel disable
				DMA1->IFCR |= (1<<17); // CTCIF5: Channel 5 transfer complete clear
				if(buffer_pos < BUFFER_LENGTH)
				{
					buffer_bluetooth_rx[buffer_pos] = bluetooth_rx;
				}
				else
				{
					buffer_pos = 0;
					buffer_bluetooth_rx[buffer_pos] = bluetooth_rx;
				}
				if(motor1.Flags.control_method == 1)
				{
					if(buffer_bluetooth_rx[(buffer_pos-1 == -1 ? (BUFFER_LENGTH -1):(buffer_pos-1))] == 'O' && buffer_bluetooth_rx[buffer_pos] == 'o')
					{
						motor1.Flags.direction ^=1; // toggle direction of motor1 rotation
					    // Set all elements of buffer_bluetooth_rx[] to 0
					    //memset(buffer_bluetooth_rx, 0, sizeof(buffer_bluetooth_rx));
					}
					else if(buffer_bluetooth_rx[(buffer_pos-1 == -1 ? (BUFFER_LENGTH -1):(buffer_pos-1))] == 'X' && buffer_bluetooth_rx[buffer_pos] == 'x')
					{
						motor1.Flags.stop_flag ^=1; // toggle ON/OFF motor1
					    // Set all elements of buffer_bluetooth_rx[] to 0
					    //memset(buffer_bluetooth_rx, 0, sizeof(buffer_bluetooth_rx));
					}
					else if(buffer_bluetooth_rx[(buffer_pos-1 == -1 ? (BUFFER_LENGTH -1):(buffer_pos-1))] == 'C' && buffer_bluetooth_rx[buffer_pos] == 'c')
					{
						motor1.Flags.control_method ^=1;
						// Set all elements of buffer_bluetooth_rx[] to 0
						//memset(buffer_bluetooth_rx, 0, sizeof(buffer_bluetooth_rx));
						if(motor1.Flags.control_method == 0)
						{
							USART1->CR1 &= ~(1<<13); // Bit 13 UE: USART Disable
							ADC1->CR2 &=~1; // Disable ADC and go to power down mode
							DMA1_Channel1->CCR &= ~1; // DMA channel 1 disable

							ADC1->CR1 |= (1<<8); // Bit 8 SCAN: Scan mode 1: Scan mode enabled
							ADC1->CR2 |= (1<<20); //Bit 20 EXTTRIG: External trigger conversion mode for regular channels
							ADC1->CR2 |= (1<<19); //Bits 19:17 EXTSEL[2:0]: External event select for regular group 100: Timer 3 TRGO event
							ADC1->CR2 |= (1<<8); //Bit 8 DMA: Direct memory access mode 1: DMA mode enabled
							ADC1->SQR1 = (1<<20); // Bits 23:20 L[3:0]: Regular channel sequence length 0010: 2 conversions
							ADC1->SQR3 = 0;
							ADC1->SQR3 |= 2;
							ADC1->SQR3 |= (3<<5);
							//ADC1->SQR3 |= (4<<10); //ADC regular sequence
							ADC1->CR2 |= 1; //ADC1 wake up
							uint32_t i;
							i=0;
							while( i < 100000) i++; //delay between power up and start of conversion in ADC1
							ADC1->CR2 |= 1; //ADC1 Start of Conversion

							DMA1_Channel1->CCR |= (1<<7); //Bit 7 MINC: Memory increment mode 1: Memory increment mode enabled
							DMA1_Channel1->CNDTR =3; //Number of data to transfer
							DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
							DMA1_Channel1->CMAR = (uint32_t)&(motor1.speed);
							DMA1_Channel1->CCR |= 1; // DMA channel 1 enable
						}
						else
						{
							ADC1->CR2 &=~1; // Disable ADC and go to power down mode
							USART1->CR1 |= (1<<13); // Bit 13 UE: USART enable

							DMA1_Channel1->CCR &= ~1; // DMA channel 1 disable

							ADC1->SQR1 = 0; // Bits 23:20 L[3:0]: Regular channel sequence length 0000: 1 conversions
							ADC1->SQR3 = 0;
							ADC1->SQR3 |= 4; //ADC regular sequence
							ADC1->CR2 |= 1; //ADC1 wake up
							uint32_t i;
							i=0;
							while( i < 100000) i++; //delay between power up and start of conversion in ADC1
							ADC1->CR2 |= 1; //ADC1 Start of Conversion

							DMA1_Channel1->CCR &= ~(1<<7); //Bit 7 MINC: Memory increment mode
							DMA1_Channel1->CNDTR =1; //Number of data to transfer
							DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
							DMA1_Channel1->CMAR = (uint32_t)&(motor1.temperature);
							DMA1_Channel1->CCR |= 1; // DMA channel 1 enable
						}
					}
					else if( buffer_bluetooth_rx[buffer_pos] == 'A')
					{
						motor1.speed = 0;
						int i=1;
						while(buffer_bluetooth_rx[((buffer_pos-i)<0)?(BUFFER_LENGTH +(buffer_pos-i)):(buffer_pos-i)] != 'a')
						{
							i++;
						}
						for(int ii=0; ii<(i-1); ii++)
						{
							motor1.speed += (((buffer_pos-(ii+1))<0)?
									        (((uint16_t)buffer_bluetooth_rx[BUFFER_LENGTH +(buffer_pos-(ii+1))])-48):
											(((uint16_t)buffer_bluetooth_rx[(buffer_pos-(ii+1))])-48))
											*pow(10,ii);
						}
						// Set all elements of buffer_bluetooth_rx[] to 0
						//memset(buffer_bluetooth_rx, 0, sizeof(buffer_bluetooth_rx));
					}
					else if( buffer_bluetooth_rx[buffer_pos] == 'B')
					{
						motor1.torque = 0;
						int i=1;
						while(buffer_bluetooth_rx[((buffer_pos-i)<0)?(BUFFER_LENGTH +(buffer_pos-i)):(buffer_pos-i)] != 'b')
						{
							i++;
						}
						for(int ii=0; ii<(i-1); ii++)
						{
							motor1.torque += (((buffer_pos-(ii+1))<0)?
						                    (((uint16_t)buffer_bluetooth_rx[BUFFER_LENGTH +(buffer_pos-(ii+1))])-48):
											(((uint16_t)buffer_bluetooth_rx[(buffer_pos-(ii+1))])-48))
											*pow(10,ii);
						}
						// Set all elements of buffer_bluetooth_rx[] to 0
						//memset(buffer_bluetooth_rx, 0, sizeof(buffer_bluetooth_rx));
					}
				}
				buffer_pos++;
				DMA1_Channel5->CCR |= 1; //Bit 0 EN: Channel enable 1: Channel enabled
			}
}

void USART1_IRQHandler(void)
{
	if((USART1->SR & (1<<7)) == (1<<7)) // Bit 7 TXE: Transmit data register empty
	{
		USART1->CR1 &= ~(1<<7); // Bit 7 TXEIE: TXE interrupt disable
		USART1->SR &= ~(1<<7); // Clear pending bit
		char str1[BUFFER_LENGTH] = "*T";
		char str2[5];
		sprintf(str2, "%d",((motor1.temperature *(110-0))/(4095-0)-26));   // Indicator is from -10 to 100
		char str3[] = "\n*";
		strcat(str1, str2);
		strcat(str1, str3);
		memcpy(buffer_bluetooth_tx, str1 , sizeof(str1));
		DMA1_Channel4->CCR |= 1; //Channel enable
		if(((motor1.temperature *(110-0))/(4095-0)-26) >= 34)
		{
//			RCC->APB1ENR |= RCC_APB1ENR_PWREN;
//			PWR->CR |= PWR_CR_PDDS; //enable standby mode
//			//PWR->CR |= PWR_CR_LPDS; //stop mode
//			PWR->CSR |= PWR_CSR_EWUP; //enable WKUP pin
//			PWR->CSR &= ~PWR_CSR_WUF; // Clear Wakeup flag
//			SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; //Enable the Standby mode by setting the SLEEPDEEP bit in the Cortex-M3 System Control Register (SCR)
//
//			PWR->CR |= (1<<8); // 1: Access to RTC and Backup registers enabled
//			while((RTC->CRL &(1<<5)) != (1<<5)); // Poll RTOFF, wait until its value goes to 1.
//			RCC->BDCR &= ~(1<<15); //Bit 15 RTCEN: RTC clock disable
//			RCC->BDCR &=~((1<<9));
//			while((RTC->CRL &(1<<5)) == (0<<5)); // Poll RTOFF, wait until its value goes to 1
//			PWR->CR &= ~(1<<8); //Disable write access to the backup registers by clearing the DBP bit in the PWR_CR register to prevent accidental writes.
//
//			NVIC ->ICER[0] = 0xFFFFFFFF;
//			NVIC ->ICER[1] = 0xFFFFFFFF;
//		    // Clear all pending interrupt bits by setting the Pending Register (ICPR) to 0xFFFFFFFF
//		    NVIC->ICPR[0] = 0xFFFFFFFF;
//		    NVIC->ICPR[1] = 0xFFFFFFFF;
//		    __DSB();
//			__WFI();
//			__ISB();
			//PWR->CSR |= PWR_CSR_EWUP; //enable WKUP pin
			HAL_PWR_EnterSTANDBYMode();
		}
	}
}
void DMA1_Channel4_IRQHandler(void)
{
	if ((DMA1->ISR &(1<<13)) == (1<<13))  //TCIF4: Channel 4 transfer complete flag
	{
		DMA1->IFCR |= (1<<13); // CTCIF4: Channel 4 transfer complete clear
		DMA1_Channel4->CCR &= ~1; //Channel disable
		NVIC ->ISER[1] |= (1<<5); // Enable global interrupt on USART1       I DONT KNOW WHY But it works. Without it NVIC for USART turn off without any command and i dont know why so i need to turn it off back
		USART1->CR1 |= (1<<7); // Bit 7 TXEIE: TXE interrupt enable
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
