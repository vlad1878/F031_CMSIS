#include "CMSIS_F031.h"

volatile uint32_t SysTimer_ms = 0;
volatile uint32_t Delay_counter_ms = 0;

extern volatile uint16_t ADC_RAW_Data[3];


void CMSIS_SW_Debug_Init(void){
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);  //I/O port A clock enabled
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_DBGMCUEN);  //MCU debug module enabled
}

void CMSIS_I2C1_PIN_Init(void){
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);  //I/O port A clock enabled
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER9_Msk, 0b10 << GPIO_MODER_MODER9_Pos);  //Alternate function mode
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER10_Msk, 0b10 << GPIO_MODER_MODER10_Pos);  //Alternate function mode
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEEDR9_Msk, 0b11 << GPIO_OSPEEDR_OSPEEDR9_Pos);  //High speed
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEEDR10_Msk, 0b11 << GPIO_OSPEEDR_OSPEEDR10_Pos);  //High speed
	MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFRH2_Msk, 0b0100 << GPIO_AFRH_AFRH2_Pos);  //I2C1_SCL PA9
	MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFRH3_Msk, 0b0100 << GPIO_AFRH_AFRH3_Pos);  //I2C1_SDA PA10

}

void CMSIS_SET_PB3_OUTPUT_Push_Pull(void){
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);  //I/O port B clock enabled
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER3_Msk, 0b01 << GPIO_MODER_MODER3_Pos);  //General purpose output mode
	CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_3);  //Output push-pull
	MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEEDR3_Msk, 0b11 << GPIO_OSPEEDR_OSPEEDR3_Pos);  //High speed
}

void CMSIS_SET_PB4_OUTPUT_Push_Pull(void){
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);  //I/O port B clock enabled
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER4_Msk, 0b01 << GPIO_MODER_MODER4_Pos);  //General purpose output mode
	CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_4);   //Output push-pull
	MODIFY_REG(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEEDR4_Msk, 0b11 << GPIO_OSPEEDR_OSPEEDR4_Pos);  //High speed
}

void CMSIS_RCC_Init_48MHz(void){
	SET_BIT(RCC->CR, RCC_CR_HSION);  //HSI oscillator ON
	while((READ_BIT(RCC->CR, RCC_CR_HSIRDY)) == RESET);  //Waiting HSI oscillator ready flag
	CLEAR_BIT(RCC->CR, RCC_CR_HSEON);  //HSE oscillator OFF
	CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);  //HSE crystal oscillator not bypassed

	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, 0b10 << RCC_CFGR_SW_Pos);  //PLL selected as system clock
	MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, 0b0000 << RCC_CFGR_HPRE_Pos);  //SYSCLK not divided
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE_Msk, 0b000 << RCC_CFGR_PPRE_Pos);  //HCLK not divided
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC_Msk, 0b00 << RCC_CFGR_PLLSRC_Pos);  //HSI/2 selected as PLL input clock
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMUL_Msk, 0b1010 << RCC_CFGR_PLLMUL_Pos);  //PLL input clock x 12
	SET_BIT(RCC->CR, RCC_CR_PLLON);  //PLL ON
	while((READ_BIT(RCC->CR, RCC_CR_PLLRDY)) == RESET);  //Waiting PLL ready flag

}

void CMSIS_Sys_Tick_Init(void){
	SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk);  //Counting down to zero to asserts the SysTick exception request
	SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk);  //Processor clock
	MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, 47999 << SysTick_LOAD_RELOAD_Pos);  //1ms
	MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk, 47999 << SysTick_VAL_CURRENT_Pos);   //value start of counting
	SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);  // Counter enabled
}

void Delay_ms(uint32_t Milliseconds){
	Delay_counter_ms = Milliseconds;
	while(Delay_counter_ms != 0);
}

void SysTick_Handler(void){
	SysTimer_ms++;
	if(Delay_counter_ms){
		Delay_counter_ms--;
	}
}

void CMSIS_ADC_DMA_Init(void){
	MODIFY_REG(ADC1->CFGR2, ADC_CFGR2_CKMODE_Msk, 0b10 << ADC_CFGR2_CKMODE_Pos);  //PCLK/4 (Synchronous clock mode)
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADCEN);  //ADC interface clock enabled
	SET_BIT(ADC1->IER, ADC_IER_EOCIE);  //EOC interrupt enabled. An interrupt is generated when the EOC bit is set.
	SET_BIT(ADC1->CFGR1, ADC_CFGR1_DMAEN);  //DMA enabled
	SET_BIT(ADC1->CFGR1, ADC_CFGR1_DMACFG);  //DMA circular mode selected
	MODIFY_REG(ADC1->CFGR1, ADC_CFGR1_RES_Msk, 0b00 << ADC_CFGR1_RES_Pos);  //12 bits of resolution
	CLEAR_BIT(ADC1->CFGR1, ADC_CFGR1_ALIGN);  //Right alignment
	SET_BIT(ADC1->CFGR1, ADC_CFGR1_CONT);  //Continuous conversion mode
	CLEAR_BIT(ADC1->CFGR1, ADC_CFGR1_DISCEN);  //Discontinuous mode disabled
	MODIFY_REG(ADC1->SMPR, ADC_SMPR_SMP_Msk, 0b111 << ADC_SMPR_SMP_Pos);  //239.5 ADC clock cycles
	SET_BIT(ADC1->CHSELR, ADC_CHSELR_CHSEL1);
	SET_BIT(ADC1->CHSELR, ADC_CHSELR_CHSEL2);
	SET_BIT(ADC1->CHSELR, ADC_CHSELR_CHSEL17);
	ADC->CCR |= ADC_CCR_VREFEN;  //VREFINT enabled
	SET_BIT(ADC1->CR, ADC_CR_ADSTART);
	SET_BIT(ADC1->CR, ADC_CR_ADEN);  //ADC enable

	SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);  // DMA clock enabled
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));  //Adress periferium
	DMA1_Channel1->CMAR = (uint32_t)(ADC_RAW_Data);  //Buffer for data
	DMA1_Channel1->CNDTR = 3;  //count data for transmit
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_TCIE);  //TC interrupt enabled
	CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_DIR);  //Read from peripheral
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_CIRC);  //circular mode enable
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_MINC);  //memory increment mode
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_TEIE);  //transfer error interrupt enable
	MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_MSIZE_Msk, 0b01 << DMA_CCR_MSIZE_Pos);  //16 bits
	MODIFY_REG(DMA1_Channel1->CCR, DMA_CCR_PSIZE_Msk, 0b01 << DMA_CCR_PSIZE_Pos);  //16 bits
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_EN);  //channel enable
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void CMSIS_SET_PA1_ADC_IN1(void){
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);  //I/O port A clock enabled
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER1_Msk, 0b11 << GPIO_MODER_MODER1_Pos);  //Analog mode PA1
}

void CMSIS_SET_PA2_ADC_IN2(void){
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);  //I/O port A clock enabled
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER2_Msk, 0b11 << GPIO_MODER_MODER2_Pos);  //Analog mode PA2
}

void CMSIS_I2C1_Init(void){
		//Page 684//
	CMSIS_SET_PB9_I2C1_SCL();
	CMSIS_SET_PB10_I2C1_SDA();
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);  // I2C1 clock enabled
	CLEAR_BIT(I2C1->CR2, I2C_CR2_ADD10);  //The master operates in 7-bit addressing mode
	CLEAR_BIT(I2C1->CR2, I2C_CR2_RD_WRN);  //Master requests a write transfer
	MODIFY_REG(I2C1->CR2, I2C_CR2_SADD_Msk, 0x4e << I2C_CR2_SADD_Pos); //7-bit slave address
	I2C1->TIMINGR = (uint32_t)0x00B01A4B;  //fast Mode @400kHz
	SET_BIT(I2C1->CR2, I2C_CR2_AUTOEND); //Automatic end mode: a STOP condition is automatically sent when NBYTES data
	I2C1->CR2 = (1 << 16);
	SET_BIT(I2C1->CR1, I2C_CR1_PE);  //Periph enable
}

void CMSIS_SET_PB9_I2C1_SCL(void){
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);  //I/O port B clock enabled
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER9_Msk, 0b10 << GPIO_MODER_MODER9_Pos);  //Alternate function mode
}

void CMSIS_SET_PB10_I2C1_SDA(void){
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN);  //I/O port B clock enabled
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER10_Msk, 0b10 << GPIO_MODER_MODER10_Pos);  //Alternate function mode
}

