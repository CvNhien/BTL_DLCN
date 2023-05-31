#include "stm32f4xx.h"
#include "string.h"
#include "math.h"
#define nb_ADCValue 50
#define G_kd 	(float)21.4
#define R_1k 987.0
#define R0_value 100.0
#define A_value (float)0.00439285  
#define B_value (float)-0.00000642
#define V_dd 2.97

//UART_RX
#define		BUFF_SIZE_RX	3		// 2 Byte dau la thoi gian, BYTE con lai la ky tu dac biet
uint8_t data_Rx[BUFF_SIZE_RX] = {};

void IntToStr4(uint16_t u, uint8_t *y);
void UART_Rx_Config(void);
void UART_Tx_Config(void);
void Display1( uint8_t* Addr_txbuff, uint16_t BUFF );
void TIM2_Config(uint16_t ms);

volatile uint16_t String_ADCValue[nb_ADCValue]={0}; 
void ADC_Config(void); 
void Led_Alarm_Init(void);
void delay_us(uint16_t period);
int ATD(uint8_t A[], uint8_t i);
uint8_t String_Sample_Time[BUFF_SIZE_RX - 1] = {"0"};

#define		BUFF_SIZE_TX	4		//DO CHINH XAC 2 CHU SO SAU DAU THAP PHAN
uint8_t ADC_Value[BUFF_SIZE_TX] = {'1','2','3','4'};
uint8_t ADC_Tx_Value[BUFF_SIZE_TX +1] = {};

DMA_InitTypeDef  	DMA_InitStructure;
float  adc_value;
float  adc_value_filter;
uint16_t	adc_average = 0;
float vol;

float temp;
float temp1;
uint16_t temp2;

float ADCvalue_to_Resistance(float denta_vol);
float ADCvalue_to_Resistance(float denta_vol) // chuyen dien ap thanh dien tro
{
	vol = denta_vol/G_kd + (5.0*101.0/(987 +101.0));
	float resistance = (R_1k*vol)/(5.0 - vol);
	return resistance;
}

float PT100_Calib(float resistance_value);
float PT100_Calib(float resistance_value)
{
		float Temperture = -(2*(R0_value - resistance_value))/(R0_value*A_value +sqrt(R0_value*R0_value*A_value*A_value - 4*R0_value*B_value*(R0_value - resistance_value)));
	  return Temperture;
}

typedef struct
{
	float Last_P;
	float Now_P;
	float out;
	float Kg;
	float Q;
	float R;
}Kalman;

Kalman kfp_instance;
void Kalman_Init(Kalman* kfp)
{
	kfp->Last_P = 1;
	kfp->Now_P = 0;
	kfp->out = 0;
	kfp->Kg = 0;
	kfp->Q = 0;
	kfp->R = 0.01;
}

float KalmanFilter(Kalman *kfp, float input)
{
	kfp->Now_P = kfp->Last_P + kfp->Q;
	kfp->Kg = kfp->Now_P/( kfp->Now_P+ kfp->R);
	kfp->out = kfp->out + kfp->Kg*(input - kfp->out);
	kfp->Last_P = (1-kfp->Kg)*kfp->Now_P;
	return kfp->out;
}
	
uint16_t average() {
    int sum = 0;
    for (int i = 0; i < nb_ADCValue; i++) {
        sum += String_ADCValue[i];
    }
    return sum / nb_ADCValue;
}


int main(void)
{
	ADC_Config();
	Led_Alarm_Init();
	UART_Rx_Config();
	Kalman_Init(&kfp_instance);
	//UART_Tx_Config();
	//TIM2_Config(1);
	while(1)
		{
		delay_us(10000);
		adc_average = (uint16_t)(0xfff &average());
		adc_value = V_dd *((float)adc_average)/4095;
		adc_value_filter = KalmanFilter(&kfp_instance, adc_value);
		}
}

void ADC_Config(void) // debug xem can chinh gia tri trong thanh ghi data
{
/*
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;	
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
TIM_TimeBaseStructure.TIM_Period = 8400 - 1; // T?n s? 10 KHz
TIM_TimeBaseStructure.TIM_Prescaler = 10;
TIM_TimeBaseStructure.TIM_ClockDivision = 0;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
*/
	
DMA_InitTypeDef DMA_InitStructure; 
ADC_InitTypeDef ADC_InitStructure; 
ADC_CommonInitTypeDef ADC_CommonInitStructure; 
GPIO_InitTypeDef GPIO_InitStructure; 
	
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); 
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2; 
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; /*Chon mode analog*/ 
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
GPIO_Init(GPIOC, &GPIO_InitStructure);
	//////////////////
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
GPIO_InitStructure.GPIO_Pin =GPIO_Pin_2; 
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; /*Chon mode analog*/ 
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
GPIO_Init(GPIOA, &GPIO_InitStructure);
	
DMA_InitStructure.DMA_Channel = DMA_Channel_0; /*chanel duoc ho tro la chanel 0-do bang*/ 
DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&String_ADCValue; /*ghep bien DMA_Memory0BaseAddr chua dia chi va cung kieu voi bien ADCValue*/ 
DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC1->DR)); /*gan dia chi thanh ghi chua gia tri chuyen doi ADC vao bien DMA_PeripheralBaseAddr 
cua DMA*/ 
DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; /*chon huong chuyen du lieu*/
DMA_InitStructure.DMA_BufferSize = nb_ADCValue; /*chon kich thuoc mang du lieu*/ 
DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; /*moi lan chuyen du lieu, dia chi ngoai vi se ko tang dan*/ 
DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; /*moi khi chuyen du lieu can tang dia chi bo nho*/ 
DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; /* kich thuoc thanh ghi chua du lieu ngoai vi la 16bit*/ 
DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; /*kich thuoc mang du lieu ADCValue là 16bit*/ 
DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; /*chon mode DMA vong tron, viec chuyen doi lien tuc lap lao*/ 
DMA_InitStructure.DMA_Priority = DMA_Priority_High; /*thiet lap che do uu tien cao*/ 
DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable; 
DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
DMA_Init(DMA2_Stream0, &DMA_InitStructure); 
/* DMA2_Stream0 enable */ 
DMA_Cmd(DMA2_Stream0, ENABLE); 
/*
NVIC_InitTypeDef  NVIC_InitStructure_0;	
NVIC_InitStructure_0.NVIC_IRQChannel = DMA2_Stream0_IRQn;
NVIC_InitStructure_0.NVIC_IRQChannelPreemptionPriority = 1;
NVIC_InitStructure_0.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure_0.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure_0);
*/  /* Transfer complete interrupt mask */
//DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);


/* ADC Common Init 
**********************************************************/ 
ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; /*chon mode Independent, Dual, Triple cho ADC*/ 
ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; /*thiet lap bo chia 2, cho ADC lay mau o tan so cao nhat*/ 
ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; /*Configures the Direct memory access mode for multi ADC mode */
ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_10Cycles; /*thoi gia tre giua 2 lan lay mau (5-20 chu ky)*/ 
ADC_CommonInit(&ADC_CommonInitStructure); 

ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; /*che do phan giai ADC la 12 bit*/

ADC_InitStructure.ADC_ScanConvMode = DISABLE;//ENABLE; 
ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //neu DISABLE thì chi nhan gia tri ADC, 1 lan duy dat khong thay doi

ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//ADC1->CR2 &= ~(1<<11);
ADC_InitStructure.ADC_NbrOfConversion = 1; /*so kenh ADC chuyen doi*/ 
ADC_Init(ADC1, &ADC_InitStructure); 
/* ADC1 regular channels configuration */ 
//ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles); 
//ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_3Cycles); 
//ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_112Cycles); 
ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_480Cycles); 
/* Enable ADC1 DMA */ 
ADC_DMACmd(ADC1, ENABLE); 
/* Enable DMA request after last transfer (Single-ADC mode) */ 
ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE); 
/* Enable ADC1 */ 
ADC_Cmd(ADC1, ENABLE); 
/* Start ADC1 Software Conversion */
ADC_SoftwareStartConv(ADC1);

/*
ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
NVIC_InitTypeDef  NVIC_InitStructure;	
NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
*/
}

/*
void ADC_IRQHandler(void)
{
	//GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
 if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
    {
			adc_value = (int16_t)(0xFFF & ADC_GetConversionValue(ADC1));
		}
	ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
}
*/
/*
void DMA2_Stream0_IRQHandler(void)	//DMA ADC
{
	//GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
}
*/
void Led_Alarm_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14|GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}
void delay_us(uint16_t period)
{
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM6->PSC = 83;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 1MHz
  	TIM6->ARR = period-1;
  	TIM6->CNT = 0;
  	TIM6->EGR = 1;		// update registers;

  	TIM6->SR  = 0;		// clear overflow flag
  	TIM6->CR1 = 1;		// enable Timer6

  	while (!TIM6->SR);
    
  	TIM6->CR1 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}
void UART_Rx_Config(void)	// PC10 PC11
{
	GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;  
	DMA_InitTypeDef   DMA_InitStructure_Rx;
  NVIC_InitTypeDef  NVIC_InitStructure;	
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);// DMA CO DOI KHONG??
  /* Connect UART4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);//
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4); //
  /* GPIO Configuration for UART4 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;//
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* GPIO Configuration for USART Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &USART_InitStructure);
  /* Enable USART */
  USART_Cmd(UART4, ENABLE);
	/* Enable UART4 DMA */
  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	/* DMA1 Stream2 Channel4 for USART4 Rx configuration */			
  DMA_InitStructure_Rx.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure_Rx.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure_Rx.DMA_Memory0BaseAddr = (uint32_t)data_Rx;
  DMA_InitStructure_Rx.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure_Rx.DMA_BufferSize = BUFF_SIZE_RX; //
  DMA_InitStructure_Rx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure_Rx.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure_Rx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure_Rx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure_Rx.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
  DMA_InitStructure_Rx.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure_Rx.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure_Rx.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure_Rx.DMA_MemoryBurst = DMA_MemoryBurst_INC8;
  DMA_InitStructure_Rx.DMA_PeripheralBurst = DMA_PeripheralBurst_INC8;
  DMA_Init(DMA1_Stream2, &DMA_InitStructure_Rx);
  DMA_Cmd(DMA1_Stream2, ENABLE);
	/* Enable DMA Interrupt to the highest priority */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Transfer complete interrupt mask */
  DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
}
void DMA1_Stream2_IRQHandler(void)	
{
	
  DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
	DMA_Cmd(DMA1_Stream2, ENABLE);
	ADC1->SR &= ~(1<<5);

	if(!(data_Rx[2] - 0x2A))
	{
			for(int i =0; i < BUFF_SIZE_RX - 1; i++)
		{
			String_Sample_Time[i] = data_Rx[i];
		}
		//ADC_Config();
		TIM2_Config(10);
		TIM2_Config(ATD(String_Sample_Time, 2));
		UART_Tx_Config();
	}
	else if(!(data_Rx[2] - 0x41))
	{
		for(int i =0; i < BUFF_SIZE_RX - 1; i++)
		{
			String_Sample_Time[i] = data_Rx[i];
		}
		//ADC_Config();
		TIM2_Config(10);
		TIM2_Config(ATD(String_Sample_Time, 2));
		UART_Tx_Config();
		// Ðèn canh bao
		if(!(data_Rx[0] - 0x31))
		{	
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
			GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		}	
	  else if(!(data_Rx[1] - 0x31))
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_14);
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		}
		else
		{
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		}	
		
		TIM_DeInit(TIM2);
	}
	else
	{
		//ADC_DeInit();
		TIM_DeInit(TIM2);
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	}

}
void Display1( uint8_t* Addr_txbuff, uint16_t BUFF )
{
		//Tach 1 phan tu ham UART_Tx_Config ra
	  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Addr_txbuff;
		DMA_InitStructure.DMA_BufferSize = BUFF;//BUFF_SIZE_UART;
	  DMA_Init(DMA1_Stream4, &DMA_InitStructure);
		DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
	  DMA_Cmd(DMA1_Stream4, ENABLE);		// phai cho phep truyen lai
		while(DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4)  == RESET );	
}

void UART_Tx_Config(void)
{
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;   
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  /* Connect UART4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);//
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4); //
  /* GPIO Configuration for UART4 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;//
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);//
  /* GPIO Configuration for USART Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;//
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOC, &GPIO_InitStructure);//
       
  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &USART_InitStructure);
	/* Enable USART */
  USART_Cmd(UART4, ENABLE);
	/* Enable UART4 DMA */
  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE); 
		/* DMA1 Stream4 Channel4 for UART4 Tx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
}

void TIM2_Config(uint16_t ms)	// Ngat de gui tinh hieu len GUI
{
	NVIC_InitTypeDef  NVIC_InitStructure;	
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	/*TIMER Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  
	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_Period = (100000* ms)-1;          
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/* TIMER TRGO selection */
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update); 
	/* TIMER enable counter */
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);/*thiet lap ngat khi tran bo nho co thong so TIM_IT_Update*/ 
	TIM_Cmd(TIM2, ENABLE); 
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler(void) 
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) 
	{
		temp1 = ADCvalue_to_Resistance(adc_value_filter);
		temp = PT100_Calib(temp1);
		temp2 = (uint16_t)(temp*100);
		
		IntToStr4(temp2, ADC_Value);
		memcpy(ADC_Tx_Value, ADC_Value, BUFF_SIZE_TX);
		ADC_Tx_Value[BUFF_SIZE_TX]='*';
		Display1(ADC_Tx_Value, BUFF_SIZE_TX +1);	
		
	}

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
}
void IntToStr4(uint16_t u, uint8_t *y)
{
	uint16_t a;
	a = u;
	y[3] = a % 10 + 0x30;
	a = a/10;
	y[2] = a % 10 + 0x30;
	a = a/10;
	y[1] = a % 10 + 0x30;
	a = a/10;
	y[0] = a + 0x30;
}
int ATD(uint8_t A[], uint8_t i)//Chuyen chuoi sang ma ASCII
{
	uint16_t D=0;
	for(int j=0;j<i;j++)
	{
		D=D+(A[j]-0x30)*pow(10,i-1-j);
	}
	return D;
}