#include <stdint.h>
#include <system_stm32f0xx.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_adc.h>
#include "stm32f0xx_dma.h"
#include "stm32f0xx_tim.h"
#include <stm32f0xx_usart.h>
#include <stm32f0xx_misc.h>
#include <lcd.h>
#include<math.h>

#define SysTick_Frequency 6000000
#define RS_TX   GPIO_Pin_9
#define ADC1_DR_Address                0x40012440

uint16_t Data_Tab[2];
uint8_t sw,c,d,e,f;
uint8_t dan[4];
uint16_t wyn,sat;


void RS_Send(uint8_t ch)
{
   USART_SendData(USART1, ch);
   while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
}


void RS_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); /* Enable USART clock */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);   /* Connect PXx to USARTx_Tx */

    /* Configure USART Tx, Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = RS_TX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_Init(USART1, &USART_InitStructure);  /* USART configuration */
    USART_OverrunDetectionConfig(USART1, USART_OVRDetection_Disable);
    USART_Cmd(USART1, ENABLE);                 /* Enable USART */
}


void ButtInit(void)
{
	GPIO_InitTypeDef InitGpio;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    InitGpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2;
    InitGpio.GPIO_Mode = GPIO_Mode_AF;
    InitGpio.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &InitGpio);

   	InitGpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    InitGpio.GPIO_Mode = GPIO_Mode_OUT;
   	InitGpio.GPIO_Speed = GPIO_Speed_Level_1;
   	InitGpio.GPIO_OType = GPIO_OType_PP;
   	InitGpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
   	GPIO_Init(GPIOA, &InitGpio);
}



void ADC_DMA(void)
{
  ADC_InitTypeDef     ADC_InitStructure;
  DMA_InitTypeDef     DMA_InitStructure;

  ADC_DeInit(ADC1);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Data_Tab;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize =
		  DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  DMA_Cmd(DMA1_Channel1, ENABLE);
  ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
  ADC_DMACmd(ADC1, ENABLE);

  ADC_StructInit(&ADC_InitStructure);

  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_ChannelConfig(ADC1, ADC_Channel_2, ADC_SampleTime_55_5Cycles);
  ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_55_5Cycles);

  ADC_GetCalibrationFactor(ADC1);

  ADC_Cmd(ADC1, ENABLE);

  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN));

  ADC_StartOfConversion(ADC1);
}

void SysTick_Handler(void)
{
  uint16_t a;

  while((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET );
  DMA_ClearFlag(DMA1_FLAG_TC1);

  a = (uint32_t)(Data_Tab[0]);  // sk³adowa zmienna
  c = a>>4;
  a = (uint32_t)(Data_Tab[1]);  // sk³adowa sta³a
  e = a>>4;

  if (sw==0){
	GPIO_SetBits(GPIOA, (GPIO_Pin_5));
	GPIO_ResetBits(GPIOA, (GPIO_Pin_4));
	sw=1;
	d=c;
	f=e;
  }else{
    GPIO_SetBits(GPIOA, (GPIO_Pin_4));
    GPIO_ResetBits(GPIOA, (GPIO_Pin_5));
    sw=0;
    dan[0]=d;
    dan[1]=c;
    dan[2]=f;
    dan[3]=e;
    Calculate_Pulse();

    RS_Send(d);
    RS_Send(c);
  }
}


//---------------------------------------------------------------------------------------------------
uint32_t SysTick_Config_Mod(uint32_t ticks)
{
if (ticks > SysTick_LOAD_RELOAD_Msk)  return (1);

  SysTick->LOAD  = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;
  NVIC_SetPriority (SysTick_IRQn, 0);
  SysTick->VAL   = 0;
  SysTick->CTRL  = SysTick_CLKSource_HCLK_Div8 & (SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
  return (0);
}



void Calculate_Pulse(void)
{
  static int8_t diff[2][400]={0};
  static uint16_t sum_DC[2]={0};
  static double buff_peak[2]={0}, avg_DC[2]={0};
  static uint8_t  uderzenia_serca=0, licznik=0, blokada=0, ilosc_probek=1, puls=0, saturation=0, peak_to_peak[2]={0}, probka[2]={0}, pop_probka[2]={0},all_peaks=0;
  static uint8_t uderzenia_w_pomiarze = 4, potwierdzenia = 3, prog = 3;

  uint16_t sat2;
  int r=0;
  for( r=0;r<2;r++) // dla sygnalow z obydwu diod
  {

            pop_probka[r] = probka[r];               // poprzednia probka
	        probka[r] = dan[r];                     //biezaca probka
	        diff[r][ilosc_probek] = probka[r]-pop_probka[r];   // tablica pochodnych

            sum_DC[r] += dan[r+2]; // suma wszystkich wartosci DC dla danej diody

        if(diff[r][ilosc_probek]<0) // jesli funkcja jest malejaca
        {
         if(peak_to_peak[r]!=0)
         buff_peak[r]=peak_to_peak[r]; // przepisanie zsumowanej wartosci miedzyszczytowej do bufora

         peak_to_peak[r] = 0;

        }
         else
         peak_to_peak[r] += diff[r][ilosc_probek]; // zwiekszanie wartosci miedzyszczytowej

  }

	      if(diff[0][ilosc_probek]>prog) // jesli sygnal rosnie dostatecznie szybko
	      {
	         licznik++;
	      }
	      else
	      {
	        licznik=0;
	        blokada=0; // wylaczenie blokady, biezaca probka nie znajduje sie na wzgorzu
	      }

	      if(blokada==0) //jesli nie jest badany kolejny raz tego samego wzniesienia
	      if(licznik>=potwierdzenia)
	      {
	    	  if(uderzenia_serca>0)
	    	 	 uderzenia_serca++;

	    	  blokada = 1;

	          if(uderzenia_serca==0)
	          uderzenia_serca=4;
	      }

	       if((uderzenia_serca==uderzenia_w_pomiarze)||(ilosc_probek>180))
	    {
	    	uderzenia_serca = 1;
	    	all_peaks=1;
	    }

        wyn=0;

        if(diff[0][ilosc_probek]<0) // jesli sygnal jest malejacy
        if(all_peaks==1)            // jesli wykryto zadana ilosc szczytow
        {
            puls = 9000/ilosc_probek; // 50[probek/sek] * 180 [sek] / ilosc_probek[ilosc_probek]
            avg_DC[0] = sum_DC[0]/ilosc_probek; // srednia wartosc DC na czerwonej diodzie
            avg_DC[1] = sum_DC[1]/ilosc_probek; // srednia wartosc DC na podczerwonej diodzie

           sat = 100*(buff_peak[0]/avg_DC[0])/(buff_peak[1]/avg_DC[1]);
           if (sat<88)sat=88;
           if (sat>98)sat=98;
           if(ilosc_probek>110)
           if(ilosc_probek<180)
           wyn = puls;

           ilosc_probek=0;
           all_peaks=0;
        }

         ilosc_probek++; //zwiekszanie ilosci probek w pomiarze
	   // return wynik;
}


int main(void)
{
	uint32_t a,b;
	uint8_t puls;

	SystemInit();
    ButtInit();

    LCD_Init();
    LCD_Clear();
    LCD_Puts((char *)"Pulsoksymetr v10");


    RS_Init();
    ADC_DMA();

    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    if (SysTick_Config_Mod(SysTick_Frequency/100)) while(1);

    GPIO_SetBits(GPIOA, (GPIO_Pin_4));
    GPIO_ResetBits(GPIOA, (GPIO_Pin_5));

    wyn=0;
    a=0;
    b=0;
    c=0;
    sw=0;
    sat=0;

    LCD_Goto(1,2);
    LCD_Puts((char *)"Init OK");
    while(1)
    {

    	if (e>180)
    	    {
    		  a=e;
    		  if (wyn>0)
    		  {
    	    	 LCD_Clear();
    	    	 puls=wyn;
    	    	 b=puls;
    	    	 LCD_PutUnsignedInt(b);LCD_Puts((char *)" bpm [pulse]");
    	    	 puls=sat;
    	    	 b=puls;
    	    	 LCD_Goto(1,2);
    	    	 LCD_PutUnsignedInt(b);LCD_Puts((char *)"%    [satur]");

                }
    	      }
    	if ((e<181)&&(a>180))
    	      {
    	    	 LCD_Clear();
    	    	 LCD_Puts((char *)"Finger OFF");
    	    	 a=e;
    	    	 wyn=0;
    	      }
    }
}

