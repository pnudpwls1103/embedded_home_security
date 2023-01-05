#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"

#define TEMPMAX 1000
#define GASMAX  00

volatile uint16_t ADC_Value[1];

volatile int btnFlag = 0;
volatile int gasFlag = 0;
volatile int usartFlag = 0;

// char password[] = {'1', '2', '3', '4'};
// volatile int idx = 0;
// volatile int wrongflag = 0;
// volatile int wrongcnt = 0;

void RCCInit(void);
void GpioInit(void);
void EXTI_Configure(void);
void DMA_Configure(void);
void ADC_Configure(void);
void NVIC_Configure(void);
void TIM_Configure(void);
void USART1_Init(void);
void USRAT2_Init(void);

void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);

void ControlPWM(int PWM);
void SetFireAlarm(void);
void ResetFireAlarm(void);
char GetLCDNumber(uint16_t x, uint16_t y);

void RCCInit(void)
{   
        // Althernate Function IO 
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN, ENABLE);
        
        // 가스센서 ADC
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN, ENABLE);
        
        // DMA
        RCC_AHBPeriphClockCmd(RCC_AHBENR_DMA1EN, ENABLE);
        
        // PWM Digital pin
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPBEN, ENABLE);
        
        // 릴레이모듈 (부저)
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPCEN, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPDEN, ENABLE);
        
        // TIMER
        RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN, ENABLE);

        // UART TX/RX
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_IOPAEN, ENABLE);
        
        // USART1
        RCC_APB2PeriphClockCmd(RCC_APB2ENR_USART1EN, ENABLE);

        // UART2
        RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

}


void GpioInit(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        
        // 가스센서 (PA5)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        
        // 버튼 (PD11)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(GPIOD, &GPIO_InitStructure);
        
        // 펌프 - 릴레이모듈 (PC8)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        
        // TIMER3_CH3 (PB0)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        // TX1(PA9) / TX2(PA2)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        
        // RX1(PA10) / RX2(A3)
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(GPIOA, &GPIO_InitStructure);


}

void EXTI_Configure(void)
{
        EXTI_InitTypeDef EXTI_InitStructure;
        
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
        DMA_Instructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_Value[0];
        DMA_Instructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_Instructure.DMA_BufferSize = 1;
        DMA_Instructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_Instructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_Instructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        DMA_Instructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        DMA_Instructure.DMA_Mode = DMA_Mode_Circular;
        DMA_Instructure.DMA_Priority = DMA_Priority_High;
        DMA_Instructure.DMA_M2M = DMA_M2M_Disable;
        
        DMA_Init(DMA1_Channel1, &DMA_Instructure);
        DMA_Cmd(DMA1_Channel1, ENABLE);
        DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
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
        
        // 버튼 NVIC
        NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        
        // PWM Timer3 NVIC
        NVIC_EnableIRQ(TIM3_IRQn);
        NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        
        // DMA
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        
        // UART1
        NVIC_EnableIRQ(USART1_IRQn);
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        // UART2
        NVIC_EnableIRQ(USART2_IRQn);
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

}

void TIM_Configure(void)
{
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        TIM_OCInitTypeDef       TIM_OCInitStructure;
        
        // TIM3_CH3 (PWM)
        TIM_TimeBaseStructure.TIM_Period = 20000;
        TIM_TimeBaseStructure.TIM_Prescaler = 72;
        TIM_TimeBaseStructure.TIM_ClockDivision = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
        
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = 1500;
        TIM_OC3Init(TIM3, &TIM_OCInitStructure);
        
        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
        TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
        TIM_ARRPreloadConfig(TIM3, ENABLE);
        TIM_Cmd(TIM3, ENABLE);
}

void USART1_Init(void)
{
        USART_InitTypeDef USART1_InitStructure;

        USART_Cmd(USART1, ENABLE);
        
        USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART1_InitStructure.USART_StopBits = USART_StopBits_1;
        USART1_InitStructure.USART_Parity = USART_Parity_No;
        USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART1_InitStructure.USART_BaudRate = 9600;
        USART_Init(USART1, &USART1_InitStructure);
   
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART2_Init(void)
{
        USART_InitTypeDef USART2_InitStructure;

        USART_Cmd(USART2, ENABLE);
        
        USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART2_InitStructure.USART_StopBits = USART_StopBits_1;
        USART2_InitStructure.USART_Parity = USART_Parity_No;
        USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART2_InitStructure.USART_BaudRate = 9600;
        USART_Init(USART2, &USART2_InitStructure);
   
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
   
}

void USART1_IRQHandler()
{
        uint16_t word;
        if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
            word = USART_ReceiveData(USART1);
            USART_SendData(USART2, word);
            USART_ClearITPendingBit(USART1,USART_IT_RXNE);
        }
}

void USART2_IRQHandler(void)
{
        uint16_t word;
        if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){
                word = USART_ReceiveData(USART2);
                USART_SendData(USART1, word);
                if(word == (uint16_t)'1') {
                        usartFlag = 0;
                }
                else {
                        
                        usartFlag = 1;
                }
            
            USART_ClearITPendingBit(USART2,USART_IT_RXNE);
        }
}

void EXTI15_10_IRQHandler() {
    if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11) == Bit_RESET){
            btnFlag = (btnFlag == 0) ? 1 : 0;
            
        }

        EXTI_ClearITPendingBit(EXTI_Line11);
    }
}

void DMA1_Channel1_IRQHandler(void)
{
        if(DMA_GetITStatus(DMA1_IT_TC1)){
                gasFlag = (ADC_Value[0] > GASMAX) ? 1: 0;
                DMA_ClearITPendingBit(DMA1_IT_GL1);
        }
}


void ControlPWM(int PWM) {
        TIM_OCInitTypeDef       TIM_OCInitStructure;
        
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = PWM;
        TIM_OC3Init(TIM3, &TIM_OCInitStructure);
}

void SetFireAlarm()
{
        GPIO_SetBits(GPIOC, GPIO_Pin_8);
        GPIO_SetBits(GPIOC, GPIO_Pin_9);
        ControlPWM(1000);
}

void ResetFireAlarm()
{
        GPIO_ResetBits(GPIOC, GPIO_Pin_8);
        GPIO_ResetBits(GPIOC, GPIO_Pin_9);
        
        ControlPWM(2000);
}

char GetLCDNumber(uint16_t x, uint16_t y)
{
        char keypad[4][3] = {
                        { '1', '2', '3' },
                        { '4', '5', '6' },
                        { '7', '8', '9'},
                        { '*', '0', '#' },
        };
  
        // 2차원 배열 - x / 80 + 1, y / 80 + 1로 처리
        char result = 'a';
        if(x >= 999 || y >= 999) {
                result = 'a';
        }
        else{
                result = keypad[x / (80+1)][y / (80+1)];
        }

        return result;
}

void Delay(void) {
	int i;
	for (i = 0; i < 500000; i++) {}
}

int main(void)
{
        SystemInit();
        RCCInit();
        
        GpioInit();
        EXTI_Configure();
        DMA_Configure();
        ADC_Configure();
        TIM_Configure();
        USART1_Init();
        USART2_Init();
        NVIC_Configure();
         
        GPIO_ResetBits(GPIOC, GPIO_Pin_8);
        GPIO_ResetBits(GPIOC, GPIO_Pin_9);

        while(1) {
                if(usartFlag) {
                        ResetFireAlarm();
                } else {
                        if(btnFlag || gasFlag) {
                                SetFireAlarm();
                        }
                        else {
                                ResetFireAlarm();
                        }
                }
                
                Delay();
        }
}