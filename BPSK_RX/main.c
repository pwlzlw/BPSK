/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  Pawel Zalewski
  * @version V1.8.0
  * @date    19-February-2017
  * @brief   Main program body
  ******************************************************************************
  * @attention
  * This is the demodulator, it runs at 160Mhz, both APB buses run at 80Mhz, 
	* there is an additional prescaler set for ADC and hence it runst at 5Mhz.
  *
  ******************************************************************************
  */
	
	
	
//#define ARM_MATH_CM4;
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sinetable.h"
#include <math.h>
#include "arm_math.h"
//#include "arm_cortexM4lf_math.lib"

/** @addtogroup DEMODULATOR
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
typedef enum {IDLE, DEMOD, BIT, PRINT } STATE;
STATE CurrentState;
/* Private define ------------------------------------------------------------*/
#define ADC_BUFFER_SIZE 256
#define   fs             140625 //khz 
#define   halfTs 				 (1/2*fs)     //half period
#define   FoscA           5500       // quiesent point 5500
#define   fsymbol					2000
#define   APB1FREQ        90000000    //80mhz timers clock, ADC runs at 10mhz
#define   KVCOa         ((FoscA*numberofsamples)/fs) //scanning the table 
#define   samplesoverfs    ((numberofsamples)/fs)
#define   pi             (float32_t)3.1415926535897
#define 	framelength    (int)( 8*fs/fsymbol)


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t CurrentPeak = 0, OldPeak = 0, FrameSize = framelength, FrameStart,FrameStartFlag =0, UARTdone =0;
volatile uint16_t clock;
uint16_t AdcValue, TablePointer;
float32_t IN[1024] = {0.0}, X[1024] = {0.0}, Y[1024] = {0.0}, phasescale =  30.5; 
uint16_t BitDone = 0, BERcnt, PrintDone = 0, DATA[8] = {0};
float32_t InPhaseA[1024] ={0.0}, QuadPhaseA[1024]={0.0}, SineHist[1024] = {0};

float32_t KVCOA = KVCOa, v0 = 0.0,v1=0.0,v2=0.0;
float32_t I_ACCUM,Q_ACCUM;


float32_t PhaseError = 0, PhaseError_REG[1024] = {0.0}, PhaseError_Acc[1024] = {0.0};
float32_t InSquareA =0.0, QuadSquareA=0.0, InSquareB=0.0, QuadSquareB=0.0;
uint16_t n,k,j,m,u,incr;
uint16_t TBLPointerA = 0; //note this go into ISR and hence best to leave them global
uint32_t AverageAccum;
/* I Q ARM FILTER at 2FC*/
 
float32_t numLPIQ[3] ={ 1, 2 , 1};
float32_t denLPIQ[3] = { 1,-1.3815568685531616  , 0.53194397687911987  };

/* LOOP FILTER AT 6 times the above  */						
float32_t numLP[3] = { 1, 2 , 1};
float32_t denLP[3] = { 1,1.3629163503646851    ,  0.52160155773162842};
													

float32_t stateA_I = 0.0, stateB_I = 0.0, stateC_I = 0.0, LP_I_W[1024] ={0.0};
float32_t stateAB_I = 0.0, stateBB_I = 0.0, stateCB_I = 0.0, LP_I_OUT[1024] ={0.0};

float32_t stateA_Q = 0.0, stateB_Q = 0.0, stateC_Q = 0.0, LP_Q_W[1024] ={0.0};
float32_t stateAB_Q = 0.0, stateBB_Q = 0.0, stateCB_Q = 0.0, LP_Q_OUT[1024] ={0.0};

float32_t stateA_M = 0.0, stateB_M = 0.0, stateC_M = 0.0, LP_M_W[1024] ={0.0};
float32_t stateAA_M = 0.0, stateBB_M = 0.0, stateCC_M = 0.0, LP_M_OUT[1024] ={0.0};

float32_t gainA = 0.037596765905618668, gainB =0.72112947702407837            ;

/* DC block */

/* Private function prototypes -----------------------------------------------*/
static void ADC_Config(void);
static void DMA_Config(void);
static void GPIO_Config(void);
static void NVIC_Config(void);
static void DAC_Config(void);
float32_t atarect(float32_t Q, float32_t I);
void CompareSamples(uint16_t, uint16_t );
uint16_t ReadTable(uint16_t*, int);
uint16_t PrevRead = 0, CurrentRead = 0;

/* Private functions ---------------------------------------------------------*/
void DMA2_Stream0_IRQHandler(void)
{
	DMA2->LIFCR = (uint32_t)(DMA_IT_TCIF0 & 0x0F7D0F7D );
	NVIC_ClearPendingIRQ (DMA2_Stream0_IRQn);	
	//GPIOC->BSRRL |= GPIO_Pin_11;
	
	switch (CurrentState) {
		
		/* IDLE */
		case IDLE :	
			GPIOB->BSRRL |= GPIO_Pin_1;
			if (AdcValue < 1054 ) {				
				n = 0;
				j = 0;
				GPIOB->BSRRH |= GPIO_Pin_1;
				CurrentState = DEMOD;
			}
			break;
		
		case DEMOD :	
			GPIOB->BSRRL |= GPIO_Pin_15;	
			/*DEMODULATOR*/		
			if (n < FrameSize + 12) {
				GPIOC->BSRRL |= GPIO_Pin_11;
				X[n] = (float32_t)( (AdcValue ) - 1117 )/ 2358; 
				
				j = (n-1) &1023;
				
				TBLPointerA = (uint16_t)( (int16_t)TBLPointerA + KVCOA - PhaseError) & 8191;
				
				InPhaseA[n] = (float32_t)(X[n] *  (sine[TBLPointerA] -2048 )/4096); 	
				QuadPhaseA[n] = (float32_t)(X[n]* (sine[(TBLPointerA - 4096) & 8191] -2048)/4096);
				
				//I AND Q ARMS FILTERING	- 2nd order butter
				stateA_I = InPhaseA[n]*gainA - denLPIQ[1]*stateB_I - denLPIQ[2]*stateC_I;
				LP_I_W[n] =  numLPIQ[0]*stateA_I + numLPIQ[1]*stateB_I + numLPIQ[2]*stateC_I;
				stateC_I = stateB_I;
				stateB_I = stateA_I;	
					
				stateA_Q = QuadPhaseA[n]*gainA  - denLPIQ[1]*stateB_Q - denLPIQ[2]*stateC_Q;
				LP_Q_W[n] =  numLPIQ[0]*stateA_Q + numLPIQ[1]*stateB_Q + numLPIQ[2]*stateC_Q;
				stateC_Q = stateB_Q;
				stateB_Q = stateA_Q;	
					
				//MULTIPLY I AND Q FILTER RESULT
				
				Y[n] = (float32_t) (LP_I_W[n] * LP_Q_W[n]);
				
				//Loop filter, 2nd order, 6 times away to avoid unstability		
				stateA_M = Y[n]*gainB - denLP[1]*stateB_M - denLP[2]*stateC_M;
				LP_M_OUT[n] =  numLP[0]*stateA_M + numLP[1]*stateB_M + numLP[2]*stateC_M;
				stateC_M = stateB_M;
				stateB_M = stateA_M;				
				
				//this is just for sending data to matlab
				PhaseError_REG[n] = PhaseError; 
				//compute the phase error
				PhaseError =  (float32_t)( LP_M_OUT[n]*(float32_t)1464.1); // 5856.4/4  I forgot why it is divided here by 4 .. some scaling was undone or an error.
				//again this is just for data dump
				PhaseError_REG[n] += PhaseError;
				PhaseError_Acc[n] = (float32_t)TBLPointerA;
				SineHist[n] = (float32_t)sine[TBLPointerA];
							
				//output waveforms to the dac
				DAC->DHR12R1 = (uint32_t)((LP_Q_W[n]) * 8192) ;					
				DAC->DHR12R2 = (uint32_t)((LP_I_W[n]) * 8192) ;	
				//increment the index 
				n = (n+1); 
				GPIOC->BSRRH |= GPIO_Pin_11;
			} else {
				//enable next state
				GPIOB->BSRRH |= GPIO_Pin_15;
				BitDone = 0;
				j =0;
				FrameStartFlag = 0;				
				CurrentState = BIT;					
			}
			break;
			
			case BIT :			
			GPIOB->BSRRL |= GPIO_Pin_14;
			//search for the first one to offset the array 
			if (BitDone != 1 ) {
				while( LP_I_W[j] != 1 && FrameStartFlag != 1) {
					j++;
				}
				FrameStartFlag = 1;
				FrameStart = j-1;
				for(u = 0; u<8;u++) {					
					//integrate and dump filter
					for (m = 0 + FrameStart; m <94 + FrameStart;m++) {
						incr = u*94; //increment every period
						AverageAccum = AverageAccum + LP_I_W[m+incr];
					}						
					if( AverageAccum  > 170) {
						DATA[u] = 1;
					} else {
						DATA[u] = 0;
					}
					AverageAccum = 0;
				}
				BitDone = 1;
				//BitDone = 1;
				GPIOB->BSRRH |= GPIO_Pin_14;
				//if (UARTdone == 1) {
					CurrentState = PRINT;		
				//}
			//}			
			break;		
			
		/* PRINT RESULTS, COMPUTE BER */
		case PRINT :
			GPIOB->BSRRL |= GPIO_Pin_13;
		
			for ( o = 0; o < framesize_data; o++) {
				if ( DATA[o] != binarydata[o] ) {
					BERcnt++;
				}					
			}	
			BER = (float32_t)((float32_t)BERcnt/(float32_t)framesize_data); 			
			BERcnt = 0;			
			PrintDone = 1;			
			MaxPeak = 0;
			MinPeak = 0; 
			CurrentState = IDLE;	
			GPIOB->BSRRH |= GPIO_Pin_13;			
			break;
			
			
			
	}
	
	//GPIOC->BSRRH |= GPIO_Pin_11;
}



/* Private functions ---------------------------------------------------------*/
void CompareSamples(uint16_t SampleNew, uint16_t SampleOld) {
	 if(SampleNew > SampleOld && SampleNew > 0x06D6) {
			//set pin high
			GPIOC->BSRRL |= GPIO_Pin_11;	
	 } else if (SampleNew < SampleOld && SampleNew < 0x06D6) {
			//set pin low 
			GPIOC->BSRRH |= GPIO_Pin_11;	 
		} 
}
static void USART_Config(void) {		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	
	USART_InitTypeDef USART_InitStructure; 	
	USART_InitStructure.USART_BaudRate = 1843200; //for matlab sampling
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;	
	USART_Init(USART2, &USART_InitStructure);	
	//STM_EVAL_COMInit(COM1, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);
}


static void ADC_Config(void)
{
  ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	/* Enable peripheral clocks */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1 , ENABLE);
	/*common for all ADCs */
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8; //i run at 7.5Mhzmhz //5.625
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);	  	
  /* Init the ADC */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_Init(ADC1, &ADC_InitStructure);		

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles); //250khz 12+28 = 40 cycles to sample 
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);	
	ADC_DMACmd(ADC1, ENABLE);	
  ADC_Cmd(ADC1, ENABLE);	
	
}  

static void GPIO_Config (void) {	
	//input
	GPIO_InitTypeDef GPIO_InitStructure;	
	/* Enable peripheral clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);
	//USART OUT
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); 
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
	/* Configure ADC Channel_0 P0 as analog input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
	// DAC  PA03 output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	// DAC  PA05 output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	//test points at BUS C 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	//8000khz LOCK = 1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	//test points for IRQ timing
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	//test points for DEMODULATOR
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	//test points for ADC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	// B test points
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}

//send a single byte
void USART_SEND(uint8_t byte)
{
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	
	USART_SendData(USART2, byte);
}


static void DMA_Config(void)
{
	//circular mode is forbidden if DMA is not the flow controller
	// only need to worry about it for SDIO
	//check for overruns OVR in the ADC_SR register, OVRIE interrupt can be enabled
	DMA_InitTypeDef DMA_InitStructure;
	/* Enable peripheral clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA_StructInit(&DMA_InitStructure);	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; //adc1
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&AdcValue;
	DMA_InitStructure.DMA_BufferSize = 1; //only one sample
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);	//ADC to DMA	
	DMA_Cmd(DMA2_Stream0, ENABLE);	
	
} 

static void NVIC_Config(void) {
	
	// Enable DMA2 Channel Transfer Complete interrupt
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE); 
	NVIC_InitTypeDef NVIC_InitStructure;
	//Enable DMA2 channel IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
}



static void DAC_Config(void)
{
	DAC_InitTypeDef  DAC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE); //at 36Mhz      
  DAC_InitStructure.DAC_Trigger        = DAC_Trigger_None;  //this runs manualy from the IRQ   
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;    
  DAC_InitStructure.DAC_OutputBuffer   = DAC_OutputBuffer_Enable;    
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);    
  DAC_Cmd(DAC_Channel_1, ENABLE);  	
	
	//dac 2
	DAC_InitStructure.DAC_Trigger        = DAC_Trigger_None;  //this runs manualy from the IRQ   
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;    
  DAC_InitStructure.DAC_OutputBuffer   = DAC_OutputBuffer_Enable;    
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);    
  DAC_Cmd(DAC_Channel_2, ENABLE);  
	
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
	DMA_Config();
  GPIO_Config();
	ADC_Config();	
	NVIC_Config();
	DAC_Config();
	USART_Config();
  ADC_SoftwareStartConv(ADC1);
  SystemCoreClockUpdate();  
	clock = SystemCoreClock;
  /* Infinite loop */
  while (1)
  {		
		if (BitDone == 1) {		
			//UARTdone = 0;
			//GPIOC->BSRRL |= GPIO_Pin_11;
			for (int y = 0; y <564+12; y++) {				
				//signal in
				uint8_t* b = (uint8_t*) &X[y];
				USART_SEND(b[3]);
				USART_SEND(b[2]);
				USART_SEND(b[1]);
				USART_SEND(b[0]);
				//PLL
				uint8_t* b4 = (uint8_t*) &SineHist[y];
				USART_SEND(b4[3]);
				USART_SEND(b4[2]);
				USART_SEND(b4[1]);
				USART_SEND(b4[0]);				
				//phase error
				uint8_t* b3 = (uint8_t*) &PhaseError_REG[y];
				USART_SEND(b3[3]);
				USART_SEND(b3[2]);
				USART_SEND(b3[1]);
				USART_SEND(b3[0]);
				//phase accumulator
				uint8_t* b2 = (uint8_t*) &PhaseError_Acc[y];
				USART_SEND(b2[3]);
				USART_SEND(b2[2]);
				USART_SEND(b2[1]);
				USART_SEND(b2[0]);
				
			}	
			//GPIOC->BSRRH |= GPIO_Pin_11;			
			BitDone = 0;
			//UARTdone = 1;
		}	
				
		//waste time
		for (int i = 0; i<100000; i++) {};
		
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
