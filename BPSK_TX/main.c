/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
	
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
#include "sinetable.h"
#include "binarydata.h"
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define   fs              2048000

  //1.2msps  

#define   foscA           6000    // fa
#define   fsymbol         2000
#define   APB1FREQ        80000000    //APB1 and APB2 timers runs at 80MHz

#define   KVCOA           ((foscA*numberofsamples)/fs) //scanning the table


#define   period (APB1FREQ /fs)



#define   numberofsamplesindex (numberofsamples-1)

#define  cyclesSymbol (fs/fsymbol)


int n = 0,m = 0, j =0, TXflag = 0,TXBusy =0, end = 0;
int k = 1;
uint16_t *TXBuffer; //note this go into ISR and hence best to leave them global
uint16_t symbol = cyclesSymbol, VCOdrift = KVCOA;
/* Private function prototypes -----------------------------------------------*/
static void DAC_Config(void);
static void GPIO_Config(void);
static void NVIC_Config(void);
static void Timer_Config(void);
/* Private functions -----------------------------------------------*/
static void DAC_Config(void)
{
	DAC_InitTypeDef  DAC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);      
  DAC_InitStructure.DAC_Trigger        = DAC_Trigger_None;  //this runs manualy from the IRQ   
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;    
  DAC_InitStructure.DAC_OutputBuffer   = DAC_OutputBuffer_Disable;    
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);    
  DAC_Cmd(DAC_Channel_1, ENABLE);  	
}  

static void GPIO_Config (void) {	
	//input
	GPIO_InitTypeDef GPIO_InitStructure;	
	/* Enable peripheral clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);
	// PA03
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	// C10 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	// THE GREEN LED 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	//BLUE BUTTON
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
}

static void Timer_Config(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	//TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM5 ,  ENABLE);	
	/* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = (uint16_t)period;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	TIM_Cmd(TIM3, ENABLE); 	
	
 
	/* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = (uint16_t)(65535);
  TIM_TimeBaseStructure.TIM_Prescaler = 1; //
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2; //40Mhz
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_SelectOutputTrigger(TIM5, TIM_TRGOSource_Update);
	TIM_Cmd(TIM5, ENABLE); 	
}

static void NVIC_Config(void) {	
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		 
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
}
/* Timer3 interrupt */
void TIM3_IRQHandler(void)       { 
	
	// clear interrupt flag	
	TIM3->SR = (uint16_t)~TIM_IT_Update;
	
	if  (n < framesize) {
		
		//get new symbol
		if ( j == 0) {
			//read array of 1 and 0
			k = binarydata[n];
			
			switch(k) {
			//output sine backwards or forwards, this could be done by reading in reverse instead of having 
			//two sinetables, in the old version I was using DMA for this - which only supported reading the same memory adress forwards...
				case 1 :
						TXBuffer = &sine[0];
						GPIOC->BSRRL |= GPIO_Pin_10;
						break;
				case 0 :
						TXBuffer = &sineinv[0];
						GPIOC->BSRRH |= GPIO_Pin_10;
						break;
			}			
		}
		
		//output the sine
		DAC->DHR12R1 = (uint32_t)TXBuffer[m]; 
		m = (m + VCOdrift) & 2047;	 

		//keep track of where we are within the symbol
		if (j < symbol) { //the elapsed fc cycles 
			j++;    
		} else {
			n = (n + 1); //needs to be here !
			j = 0;
			m = 0;
		}
		
	} else {		
		TXBusy = 0;
		DAC->DHR12R1 = (uint32_t)2048;
		GPIOA->BSRRH = GPIO_Pin_5;;
		TIM3->DIER &= (uint16_t)~TIM_IT_Update; //disable timer 3		
	}
}

//time 5 periodic frames
void TIM5_IRQHandler(void)       {  
	TIM5->SR = (uint16_t)~TIM_IT_Update;
	if (TXBusy != 1) {
		n = 0;
		m = 0;
		TXBusy = 1;		
		TIM3->DIER |= TIM_IT_Update;
		GPIOA->BSRRL = GPIO_Pin_5;
	}
}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{	
 
 /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       files before to branch to application main.
       To reconfigure the default setting of SystemInit() function, 
       refer to system_stm32f4xx.c file */
	TXBuffer = &sine[0]; //load the table
	GPIO_Config();
	DAC_Config();
  Timer_Config();	
	NVIC_Config();	
	
     
  /* Infinite loop */
  while (1)
  {		
		//clock = SystemCoreClock;
		//DetectFreq(CurrentRead);
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
		
		
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
