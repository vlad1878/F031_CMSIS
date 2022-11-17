#include "CMSIS_F031.h"
#include "SMA_filter_lib.h"
#include "stdbool.h"

extern volatile uint32_t SysTimer_ms;
extern volatile uint32_t Delay_counter_ms;
bool led_flag = 0;
unsigned long t_led = 0;

uint16_t SMA_Filter_Buffer_1[SMA_FILTER_ORDER] = { 0, };
uint16_t SMA_Filter_Buffer_2[SMA_FILTER_ORDER] = { 0, };
uint16_t SMA_Filter_Buffer_3[SMA_FILTER_ORDER] = { 0, };
uint16_t ADC_SMA_Data[3] = {0, };
volatile uint16_t ADC_RAW_Data[3] = {0, };
volatile uint32_t Counter_DMA = 0;

void DMA1_Channel1_IRQHandler(void){
	if(READ_BIT(DMA1->ISR, DMA_ISR_TCIF1)){
		SET_BIT(DMA1->IFCR, DMA_IFCR_CGIF1);
		Counter_DMA++;
		if(Counter_DMA == 1200) {
		  		Counter_DMA = 0;
		  		ADC_SMA_Data[0] = SMA_FILTER_Get_Value(SMA_Filter_Buffer_1, &ADC_RAW_Data[0]);
		  		ADC_SMA_Data[1] = SMA_FILTER_Get_Value(SMA_Filter_Buffer_2, &ADC_RAW_Data[1]);
		  		ADC_SMA_Data[2] = SMA_FILTER_Get_Value(SMA_Filter_Buffer_3, &ADC_RAW_Data[2]);
		}
	}
	else if(READ_BIT(DMA1->ISR, DMA_ISR_TEIF1)) {
		SET_BIT(DMA1->IFCR, DMA_IFCR_CTEIF1);
	}
}

void blink_led(void){
	if (SysTimer_ms - t_led > 200){
		t_led = SysTimer_ms;
		led_flag = !led_flag;
	}
	if (led_flag) {
		WRITE_REG(GPIOB->BSRR, GPIO_BSRR_BS_3);
	} else if (!led_flag) {
		WRITE_REG(GPIOB->BSRR, GPIO_BSRR_BR_3);
	}
}

int main(void)
{
	CMSIS_SW_Debug_Init();
	CMSIS_RCC_Init_48MHz();
	CMSIS_Sys_Tick_Init();
	CMSIS_SET_PA1_ADC_IN1();
	CMSIS_SET_PA2_ADC_IN2();
	CMSIS_ADC_DMA_Init();
	CMSIS_I2C1_Init();
	Delay_ms(100);
	CMSIS_SET_PB3_OUTPUT_Push_Pull();
	CMSIS_SET_PB4_OUTPUT_Push_Pull();

	t_led = SysTimer_ms;
	while (1)
	{
		blink_led();
	}
}
