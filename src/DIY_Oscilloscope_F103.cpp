#ifdef STM32F1
#include <Arduino.h>

#define ADCPIN PA0
#define DISP_W 320
#define DISP_H 240
#define MAX_SAMPLES (1024*6) // Max samples, depends on available RAM.

float sampletime;
// Samples order in an array element -> first upper (ADC2) then lower (ADC1)
uint32_t buffer_32[MAX_SAMPLES/2];
// when incrementing a pointer we increase the stored address by the size of its data type
uint16_t *buffer = (uint16_t *)buffer_32;

void setup() {
	SerialUSB.begin();
	SerialUSB.dtr(false);
	pinMode(PC13, OUTPUT);
	// Activating clock for port A, ADC1, ADC2
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	/*
	* try 36MHz ADC, see what happens :clueless:
	* the faster the ADC, less time there will be for it to sample the voltage
	* 00: F_CPU/2	01: F_CPU/4		10: F_CPU/6		11: F_CPU/8
	*/
	// RCC->CFGR &= ~(RCC_CFGR_ADCPRE);
	// RCC->CFGR |= RCC_CFGR_ADCPRE_1;
	// ADC Pin, PA0 (ADC CH0), as input analog mode 
	GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
	GPIOB->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
	GPIOB->CRL |= GPIO_CRL_MODE0; // 50Mhz mode
	// wait for clock stablization
	delay(1);

	/* Init test signal: */
	
	/* Init ADC: */
	// adc resolution for arduino (dumb fuck)
	analogReadResolution(12);
	// ADC configs
	ADC1->SQR3 = (0 << 0); // default, can be removed
	ADC2->SQR3 = (0 << 0); // default, can be removed
	ADC1->SMPR2 &= ~(ADC_SMPR2_SMP0); // default (1.5 cycle), can be removed. If Rin > 10K then use more cycles to convert.
	/*
	*  SWSTART: This bit is set by software to start conversion and cleared by hardware as soon as 
	*  conversion starts. It starts a conversion of a group of regular channels if SWSTART is 
	*  selected as trigger event by the EXTSEL[2:0] bits.
	*  ADON: first time power on, second and onward start conversions.
	*/
	ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_SWSTART | ADC_CR2_EXTSEL | ADC_CR2_DMA;
	ADC2->CR2 |= ADC_CR2_CONT;
	ADC1->CR1 |= ADC_CR1_EOCIE | ADC_CR1_DUALMOD_2 | ADC_CR1_DUALMOD_1 | ADC_CR1_DUALMOD_0;
	ADC1->CR2 |= ADC_CR2_ADON; // Power on ADC (on first set)
	ADC2->CR2 |= ADC_CR2_ADON; // Power on ADC (on first set)
	delay(1);
	// Calibrate ADC
	ADC1->CR2 |= ADC_CR2_CAL;
	ADC2->CR2 |= ADC_CR2_CAL;
	delay(1);
	ADC1->CR2 |= ADC_CR2_ADON; // Start conversion
}
void loop() {
	/*
	* Basic workflow:
	* 1. Read input from touch or serial
	* 2. Trigger
	* 3. Fill sample buffer
	* 4. Decimate samples
	* 5. Draw screen
	*/
	sampletime = micros();
	// Note: better to use DMA but can't be arsed really
	// So, bootleg DMA goes brrr
	// should give us roughly 1.7 MSPS (1706484 samples per second) at 12MHz and less than 10kOhm input res
	for(int i=0; i<MAX_SAMPLES/2; i++) {
		while(!(ADC1->SR & ADC_SR_EOC)) {;}
		buffer_32[i] = ADC1->DR;
	}
	sampletime = (micros() - sampletime) / MAX_SAMPLES;
	for(int j=0; j<MAX_SAMPLES; j++) {
		digitalWrite(PC13, !digitalRead(PC13));
		SerialUSB.print(sampletime, 3);
		SerialUSB.print(",");
		SerialUSB.println(buffer[j]);
		delayMicroseconds(100);
	}
}
#endif