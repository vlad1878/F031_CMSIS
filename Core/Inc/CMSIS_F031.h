#include "stm32f0xx.h"

void CMSIS_SW_Debug_Init(void);
void CMSIS_SET_PB3_OUTPUT_Push_Pull(void);
void CMSIS_SET_PB4_OUTPUT_Push_Pull(void);
void CMSIS_RCC_Init_48MHz(void);
void CMSIS_Sys_Tick_Init(void);
void Delay_ms(uint32_t Milliseconds);
void SysTick_Handler(void);
void CMSIS_SET_PA1_ADC_IN1(void);
void CMSIS_SET_PA2_ADC_IN2(void);
void CMSIS_ADC_DMA_Init(void);
void CMSIS_SET_PB9_I2C1_SCL(void);
void CMSIS_SET_PB10_I2C1_SDA(void);
void CMSIS_I2C1_PIN_Init(void);
void CMSIS_I2C1_Init(void);
