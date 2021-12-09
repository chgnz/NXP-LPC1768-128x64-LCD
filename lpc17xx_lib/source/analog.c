//-------------------------------------------------------------------------------------------------
/**
	InicializĆ„ā€� ADC ieejas.
*/


//#include "lpc17xx_adc.h"
//#include "lpc17xx_pinsel.h"
#include "analog.h"

int adcValue = 0;

void initADC(void)
{
/*
	// satĆ„Ā«ram stĆ„ļæ½vokli
	for (uint8_t adc_num = 0; adc_num < IO_ADC_NUM_INPUTS; ++adc_num)
	{
		// inicializĆ„ā€�jam plates baroĆ…Ā�anas ADC ieeju filtrus
		// NB: universĆ„ļæ½lo ieeju filtrus ar Ć„Ā«stajĆ„ļæ½m inicializĆ„ā€� check_config()!
		MLIB_InitIIRFilter(&adc_filters[adc_num], IO_BOARD_VOLTAGE_FILTER_GAIN);
	}

	adc_read_timestamp = 0;
*/
	PINSEL_CFG_Type PinCfg;

	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 25;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 24;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	/* Configuration for ADC:
	 *  select: ADC channel 2
	 *  ADC conversion rate = 200KHz
	 */

	ADC_Init(LPC_ADC, 200000);
	ADC_IntConfig(LPC_ADC,ADC_ADINTEN2,ENABLE); // iesledzam interuptu
	ADC_ChannelCmd(LPC_ADC,ADC_CHANNEL_2,ENABLE); // iesleedzam nepiecieshamo kanalu
	ADC_IntConfig(LPC_ADC,ADC_ADINTEN1,ENABLE); // iesledzam interuptu
	ADC_ChannelCmd(LPC_ADC,ADC_CHANNEL_1,ENABLE); // iesleedzam nepiecieshamo kanalu
	ADC_EdgeStartConfig(LPC_ADC,ADC_START_ON_FALLING); // nav ne jausmas

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(ADC_IRQn, ((0x01<<3)|0x01));


/*
    GPIO_SetDir(0,24,0);		// SET as Input
    GPIO_SetDir(0,25,0); 		// SET as Input

	set_pin_function(0,24,1); 	// SET P0.24 as ADC
	set_pin_function(0,25,1); 	// SET P0.25 as ADC

	ADC_Init(IO_ADC_PERIPH, IO_ADC_CONVERSION_RATE);

	// palaiĆ…Ā¾am ADC
//	ADC_ChannelCmd(IO_ADC_PERIPH, ADC_CHANNEL_0, DISABLE);
//	ADC_ChannelCmd(IO_ADC_PERIPH, IO_ADC_CHANNEL, ENABLE);
*/
}

void readADC()
{
	adcValue = 0;
	/* Start conversion on EINT0 falling edge */
	// ADC_StartCmd(LPC_ADC,ADC_START_ON_EINT0);
	ADC_StartCmd(LPC_ADC,ADC_START_NOW);
//	ADC_StartCmd(LPC_ADC,ADC_START_CONTINUOUS);

	/* Enable ADC in NVIC */
//	NVIC_EnableIRQ(ADC_IRQn);
}

/*
 * @brief    ADC interrupt handler routine, get the ADC value and disable the ADC interrupt.
 * @param    None.
 * @return   None.
 */

void ADC_IRQHandler(void)
{
	adcValue = 0;
	/*
	if (ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_2,ADC_DATA_DONE))
	{
		adcValue = ADC_ChannelGetData(LPC_ADC,ADC_CHANNEL_2);
		// NVIC_DisableIRQ(ADC_IRQn);
	}
	*/
	if (ADC_ChannelGetStatus(LPC_ADC,ADC_CHANNEL_1,ADC_DATA_DONE))
	{
		adcValue = ADC_ChannelGetData(LPC_ADC,ADC_CHANNEL_1);
		// NVIC_DisableIRQ(ADC_IRQn);
	}
}
