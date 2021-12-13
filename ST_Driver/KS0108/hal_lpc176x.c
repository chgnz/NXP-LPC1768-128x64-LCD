#include "hal.h"

#include "lpc17xx_GPIO.h"

#include <stdint.h>
#include <stdbool.h>

void KS0108_WRITE_DATA(uint8_t data)
{
	GPIO_SetValue(1, (data << 19));
	GPIO_ClearValue(1, (((uint8_t)~data) << 19));
}

uint8_t KS0108_READ_DATA()
{
	return (uint8_t)(GPIO_ReadValue(1) >> 19) & 0xff;
}
