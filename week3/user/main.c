#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"

// volatile unsigned 32bits
volatile uint32_t ADC_Value[2];

/* function prototype */
void RCCInit(void);
void GpioInit(void);
void EXTI_Configure(void);
void DMA_Configure(void);
void ADC_Configure(void);
void NVIC_Configure(void);

void EXTI1_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void Delay(void);

int sensorFlag = 0;
int btnFlag = 0;
void RCCInit(void)
{	
        // 가스센서 ADC
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBENR_DMA1EN, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN, ENABLE);
        
        // 인체감지센서 Digital pin
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPBEN, ENABLE);
        
        // 릴레이모듈 (부저)
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPCEN, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPDEN, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}


void GpioInit(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        
        // 가스센서 (PB5)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        
        // 인체감지센서 (PB1)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        
        // 버튼 (PD11)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(GPIOD, &GPIO_InitStructure);
        
        // 펌프 - 릴레이모듈 (PC8)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        
        // 블루투스 - 릴레이모듈 (PC9)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void EXTI_Configure(void)
{
        EXTI_InitTypeDef EXTI_InitStructure;
        
        // 인체감지센서 EXTI (PB1)
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
        EXTI_InitStructure.EXTI_Line = EXTI_Line1;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);
        
        // 버튼 EXTI (PD11)
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource11);
        EXTI_InitStructure.EXTI_Line = EXTI_Line11;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);
}
void DMA_Configure(void) {
        DMA_InitTypeDef DMA_Instructure;
        
        DMA_Instructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
        DMA_Instructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_Value[1];
        DMA_Instructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_Instructure.DMA_BufferSize = 1;
        DMA_Instructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_Instructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_Instructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        DMA_Instructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        DMA_Instructure.DMA_Mode = DMA_Mode_Circular;
        DMA_Instructure.DMA_Priority = DMA_Priority_High;
        DMA_Instructure.DMA_M2M = DMA_M2M_Enable;
        
        DMA_Init(DMA1_Channel1, &DMA_Instructure);
        DMA_Cmd(DMA1_Channel1, ENABLE);

}

void ADC_Configure(void)
{
        ADC_InitTypeDef ADC_InitStructure;
        
        ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
        ADC_InitStructure.ADC_ScanConvMode = ENABLE;
        ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
        ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
        ADC_InitStructure.ADC_NbrOfChannel = 1;
      
        ADC_Init(ADC1, &ADC_InitStructure);
        
        ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_41Cycles5);
        
        ADC_DMACmd(ADC1, ENABLE);
        
        ADC_Cmd(ADC1, ENABLE);
        
        ADC_ResetCalibration(ADC1);
        
        while(ADC_GetResetCalibrationStatus(ADC1)) {}
        
        ADC_StartCalibration(ADC1);
        
        while(ADC_GetCalibrationStatus(ADC1)) {}
        
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        
}

void NVIC_Configure(void)
{
        NVIC_InitTypeDef NVIC_InitStructure;

        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
        
        // 인체감지센서 NVIC
        NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        
        // 버튼 NVIC
        NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
}

void EXTI1_IRQHandler() {
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_SET) {
            sensorFlag = 1;
        }
        else {
            sensorFlag = 0;
        }
        EXTI_ClearITPendingBit(EXTI_Line1);
    }


}

void EXTI15_10_IRQHandler() {
    if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11) == Bit_RESET) {
            btnFlag = (btnFlag == 0) ? 1 : 0;
        }

        EXTI_ClearITPendingBit(EXTI_Line11);
    }


}

int main(void)
{
  	SystemInit();
        RCCInit();
        GpioInit();
        EXTI_Configure();
        DMA_Configure();
        ADC_Configure();
        NVIC_Configure();
        
        GPIO_ResetBits(GPIOC, GPIO_Pin_8);
        GPIO_ResetBits(GPIOC, GPIO_Pin_9);
        
	LCD_Init();
	Touch_Configuration();
	Touch_Adjust();
	LCD_Clear(WHITE);	       
        
        LCD_ShowString(80, 120, "Gas: ", BLACK, WHITE);
        LCD_ShowString(80, 140, "Motion: ", BLACK, WHITE);
        LCD_ShowString(80, 160, "Button: ", BLACK, WHITE);
        while(1) {
                if(btnFlag)
                {
                    GPIO_SetBits(GPIOC, GPIO_Pin_8);
                    GPIO_SetBits(GPIOC, GPIO_Pin_9);
                }
                else
                {
                    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
                    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
                }
                
                LCD_ShowNum(100, 120, ADC_Value[1], 10, BLACK, WHITE);
                LCD_ShowNum(100, 140, sensorFlag, 10, BLACK, WHITE);
                LCD_ShowNum(100, 160, btnFlag, 10, RED, WHITE);
                
                
	}
}